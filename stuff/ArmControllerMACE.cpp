#include "ArmControllerMACE.h"
#include "learning/MACETrainer.h"
#include "SimArm.h"

cArmControllerMACE::cArmControllerMACE()
{
	mNumActionFrags = 0;
	mExpCritic = false;
	mExpActor = false;
}

cArmControllerMACE::~cArmControllerMACE()
{
}


void cArmControllerMACE::Reset()
{
	cArmNNController::Reset();
	mExpCritic = false;
	mExpActor = false;
}

int cArmControllerMACE::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cArmControllerMACE::GetActionFragSize() const
{
	return cArmNNController::GetPoliActionSize();
}

int cArmControllerMACE::GetNetOutputSize() const
{
	return GetNumActionFrags() + GetNumActionFrags() * GetActionFragSize();
}

void cArmControllerMACE::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetPoliActionSize() + 1);

	int a = mPoliAction.mID;
	cMACETrainer::SetActionFragIdx(a, out_action);

	Eigen::VectorXd frag = mPoliAction.mParams;
	cMACETrainer::SetActionFrag(frag, out_action);
}

bool cArmControllerMACE::IsExpCritic() const
{
	return mExpCritic;
}

bool cArmControllerMACE::IsExpActor() const
{
	return mExpActor;
}

void cArmControllerMACE::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetNetOutputSize();
	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	int num_actions = GetNumActionFrags();
	int action_size = GetActionFragSize();

	out_offset.segment(0, num_actions) *= -0.5;
	out_scale.segment(0, num_actions) *= 2;

	Eigen::VectorXd action_offset;
	Eigen::VectorXd action_scale;
	cArmNNController::BuildNNOutputOffsetScale(action_offset, action_scale);

	for (int a = 0; a < num_actions; ++a)
	{
		Eigen::VectorXd curr_offset;
		BuildActorBias(a, curr_offset);
		out_offset.segment(num_actions + a * action_size, action_size) = curr_offset;
		out_scale.segment(num_actions + a * action_size, action_size) = action_scale;
	}
}

void cArmControllerMACE::BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const
{
	double min = -20;
	double max = 20;
	
	out_bias = Eigen::VectorXd::Ones(GetActionFragSize());
	int num_actors = GetNumActionFrags();
	if (num_actors < 2)
	{
		out_bias *= 0;
	}
	else
	{
		double lerp = 1 - a_id / (num_actors - 1.0);
		out_bias *= lerp * (max - min) + min;
	}
}

void cArmControllerMACE::DecideAction()
{
	mOffPolicy = false;
	mExpCritic = false;
	mExpActor = false;

	Eigen::VectorXd y;
	mNet.Eval(mPoliState, y);

	int a_max = GetMaxFragIdx(y);
	int a = a_max;

	if (mEnableExp && mExpTemp != 0)
	{
		int num_actors = GetNumActionFrags();
		double max_val = GetVal(y, a_max);

		double sum = 0;
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = GetVal(y, i);
			curr_val = std::exp((curr_val - max_val) / mExpTemp);

			mBoltzmannBuffer[i] = curr_val;
			sum += curr_val;
		}

		double rand = cMathUtil::RandDouble(0, sum);

		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = mBoltzmannBuffer[i];
			rand -= curr_val;

			if (rand <= 0)
			{
				a = i;
				break;
			}
		}
	}

	BuildActorAction(y, a, mPoliAction);

	if (mEnableExp)
	{
		double rand_noise = cMathUtil::RandDouble();
		if (rand_noise < mExpRate)
		{
			ApplyExpNoise(mPoliAction);
			mExpActor = true;
		}

		mExpCritic = (a != a_max);
	}

	if (mExpCritic || mExpActor)
	{
		mOffPolicy = true;
	}

#if defined (ENABLE_DEBUG_PRINT)
	if (mExpCritic || mExpActor)
	{
		if (mExpActor)
		{
			printf("Actor ");
		}
		if (mExpCritic)
		{
			printf("Critic ");
		}
		printf("Exploration\n");
	}
	printf("Actor: (%i)\t", a);
	for (int i = 0; i < y.size(); ++i)
	{
		double val = GetVal(y, i);
		printf("%.3f\t", val);
	}
	printf("\n");
#endif
}

void cArmControllerMACE::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = cMACETrainer::CalcNumFrags(num_outputs, GetActionFragSize());
	mBoltzmannBuffer.resize(mNumActionFrags);
}

void cArmControllerMACE::LoadNetIntern(const std::string& net_file)
{
	cArmNNController::LoadNetIntern(net_file);
	UpdateFragParams();
}

void cArmControllerMACE::BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const
{
	out_action.mID = a_id;
	GetFrag(params, a_id, out_action.mParams);
}

void cArmControllerMACE::FetchExpNoiseScale(Eigen::VectorXd& out_noise) const
{
	const Eigen::VectorXd& nn_output_scale = mNet.GetOutputScale();
	GetFrag(nn_output_scale, 0, out_noise);
	out_noise = out_noise.cwiseInverse();
}

int cArmControllerMACE::GetMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

double cArmControllerMACE::GetMaxFragVal(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragVal(params, mNumActionFrags);
}

void cArmControllerMACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cMACETrainer::GetFrag(params, mNumActionFrags, GetActionFragSize(), a_idx, out_action);
}

void cArmControllerMACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetFrag(frag, a_idx, mNumActionFrags, GetActionFragSize(), out_params);
}

double cArmControllerMACE::GetVal(const Eigen::VectorXd& params, int a_idx) const
{
	return cMACETrainer::GetVal(params, a_idx);
}

void cArmControllerMACE::SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetVal(val, a_idx, out_params);
}