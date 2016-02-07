#include "BallControllerMACE.h"
#include "Ball.h"

#define ENABLE_BOLTZMANN_EXP

const int gActionFragSize = 1;

cBallControllerMACE::cBallControllerMACE(cBall& ball) :
	cBallController(ball)
{
	mNumActionFrags = 0;
	mExpCritic = false;
	mExpActor = false;
}

cBallControllerMACE::~cBallControllerMACE()
{
}

void cBallControllerMACE::Reset()
{
	cBallController::Reset();
	mExpCritic = false;
	mExpActor = false;
}

bool cBallControllerMACE::LoadNet(const std::string& net_file)
{
	bool succ = true;
	mNet.Clear();
	mNet.LoadNet(net_file);

	if (succ)
	{
		UpdateFragParams();
	}

	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int state_size = GetNetInputSize();
	int action_size = GetNetOutputSize();

	if (output_size != action_size)
	{
		printf("Network output dimension does not match number of actions (%i vs %i).\n", output_size, state_size);
		succ = false;
	}

	if (input_size != state_size)
	{
		printf("Network input dimension does not match state size (%i vs %i).\n", input_size, state_size);
		succ = false;
	}

	if (!succ)
	{
		mNet.Clear();
		assert(false);
	}

	return succ;
}

int cBallControllerMACE::GetActionSize() const
{
	return 1 + gActionFragSize;
}


void cBallControllerMACE::ExploreAction(tAction& out_action)
{
	mOffPolicy = true;
	tAction action;

	const double critic_exp_val = 0.6;
	const double actor_exp_val = 0.8;

	double rand = cMathUtil::RandDouble();
	if (rand < critic_exp_val)
	{
		GetRandomActionFrag(action);
		mExpCritic = true;
		mExpActor = false;
	}
	else if (rand < actor_exp_val)
	{
		CalcActionNetCont(action);
		ApplyExpNoise(action);
		mExpCritic = false;
		mExpActor = true;
	}
	else
	{
		GetRandomActionFrag(action);
		ApplyExpNoise(action);
		mExpCritic = true;
		mExpActor = true;
	}

	out_action = action;
	printf("rand action: %i, %.3f\n", action.mID, action.mDist);
}

int cBallControllerMACE::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBallControllerMACE::GetActionFragSize() const
{
	return gActionFragSize;
}

int cBallControllerMACE::GetNetOutputSize() const
{
	return mNumActionFrags + mNumActionFrags * gActionFragSize;
}

void cBallControllerMACE::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());

	int a = mCurrAction.mID;
	cMACETrainer::SetActionFragIdx(a, out_action);

	Eigen::VectorXd frag = Eigen::VectorXd::Zero(gActionFragSize);
	frag[0] = mCurrAction.mDist;
	cMACETrainer::SetActionFrag(frag, out_action);
}

cBallControllerMACE::tAction cBallControllerMACE::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
	int a = cMACETrainer::GetActionFragIdx(action_params);
	Eigen::VectorXd action_frag;
	cMACETrainer::GetActionFrag(action_params, action_frag);

	assert(action_frag.size() == 1);

	tAction action;
	action.mID = a;
	action.mDist = action_frag[0];

	return action;
}

bool cBallControllerMACE::IsExpCritic() const
{
	return mExpCritic;
}

bool cBallControllerMACE::IsExpActor() const
{
	return mExpActor;
}

void cBallControllerMACE::UpdateAction()
{
	mExpCritic = false;
	mExpActor = false;
	cBallController::UpdateAction();
}

void cBallControllerMACE::DecideAction(tAction& out_action)
{
#if defined(ENABLE_BOLTZMANN_EXP)
	DecideActionBoltzmann(out_action);
#else
	cBallController::DecideAction(out_action);
#endif
}

void cBallControllerMACE::DecideActionBoltzmann(tAction& out_action)
{
	mOffPolicy = false;

	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd y;
	mNet.Eval(state, y);

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

	BuildActorAction(y, a, out_action);

	if (mEnableExp)
	{
		double rand_noise = cMathUtil::RandDouble();
		if (rand_noise < mExpRate)
		{
			ApplyExpNoise(out_action);
			mExpActor = true;
		}

		mExpCritic = (a != a_max);
	}
	
	if (mExpCritic || mExpActor)
	{
		mOffPolicy = true;
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

	DebugPrintAction(out_action, y);
}

void cBallControllerMACE::ExploitPolicy(tAction& out_action)
{
	CalcActionNetCont(out_action);
}

void cBallControllerMACE::CalcActionNetCont(tAction& out_action)
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd y;
	mNet.Eval(state, y);

	int a = GetMaxFragIdx(y);
	BuildActorAction(y, a, out_action);

	DebugPrintAction(out_action, y);
}

void cBallControllerMACE::GetRandomActionFrag(tAction& out_action)
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd y;
	mNet.Eval(state, y);

	int max_a = GetMaxFragIdx(y);
	int a = cMathUtil::RandIntExclude(0, GetNumActionFrags(), max_a);
	Eigen::VectorXd action_frag;
	GetFrag(y, a, action_frag);

	out_action.mID = a;
	out_action.mDist = action_frag[0];
}

void cBallControllerMACE::ApplyExpNoise(tAction& out_action)
{
	int rand = cMathUtil::RandInt(0, 2);
	if (rand == 0)
	{
		AddExpActionNoise(out_action);
	}
	else
	{
		AddExpStateNoise(out_action);
	}
}

void cBallControllerMACE::AddExpActionNoise(tAction& out_action)
{
	const double dist_mean = 0;
	const double dist_stdev = 0.5;

	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);

	out_action.mDist += rand_dist;
	out_action.mDist = cMathUtil::Clamp(out_action.mDist, gMinDist, gMaxDist);
}

void cBallControllerMACE::AddExpStateNoise(tAction& out_action)
{
	const double noise_mean = 0;
	const double noise_stdev = 1;

	const std::string& noise_layer = "ip0";
	Eigen::VectorXd y;
	mNet.ForwardInjectNoisePrefilled(noise_mean, noise_stdev, noise_layer, y);
	BuildActorAction(y, out_action.mID, out_action);
}

void cBallControllerMACE::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = cMACETrainer::CalcNumFrags(num_outputs, gActionFragSize);

#if defined(ENABLE_BOLTZMANN_EXP)
	mBoltzmannBuffer.resize(mNumActionFrags);
#endif
}

void cBallControllerMACE::BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const
{
	Eigen::VectorXd action_frag;
	GetFrag(params, a_id, action_frag);
	out_action.mID = a_id;
	out_action.mDist = action_frag[0];
}

void cBallControllerMACE::DebugPrintAction(const tAction& action, const Eigen::VectorXd& params) const
{
	double val = GetVal(params, action.mID);

	printf("Value: %.3f\n", val);
	printf("Action %i (%.3f):\t", action.mID, action.mDist);

	for (int i = 0; i < static_cast<int>(params.size()); ++i)
	{
		if (i != 0)
		{
			printf(",\t");
		}
		printf("%.3f", params[i]);
	}
	printf("\n\n");
}

int cBallControllerMACE::GetMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

double cBallControllerMACE::GetMaxFragVal(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragVal(params, mNumActionFrags);
}

void cBallControllerMACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cMACETrainer::GetFrag(params, mNumActionFrags, gActionFragSize, a_idx, out_action);
}

void cBallControllerMACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetFrag(frag, a_idx, mNumActionFrags, gActionFragSize, out_params);
}

double cBallControllerMACE::GetVal(const Eigen::VectorXd& params, int a_idx) const
{
	return cMACETrainer::GetVal(params, a_idx);
}

void cBallControllerMACE::SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetVal(val, a_idx, out_params);
}