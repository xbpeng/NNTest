#include "BallControllerMACEDPG.h"
#include "Ball.h"
#include "learning/MACEDPGTrainer.h"

cBallControllerMACEDPG::cBallControllerMACEDPG(cBall& ball) :
	cBallControllerDPG(ball)
{
	mNumActionFrags = 0;
	mExpCritic = false;
	mExpActor = false;
}

cBallControllerMACEDPG::~cBallControllerMACEDPG()
{
}

void cBallControllerMACEDPG::Reset()
{
	cBallControllerDPG::Reset();
	mExpCritic = false;
	mExpActor = false;
}

int cBallControllerMACEDPG::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBallControllerMACEDPG::GetActionFragSize() const
{
	return GetActionSize();
}

int cBallControllerMACEDPG::GetNetOutputSize() const
{
	return GetActorOutputSize();
}

int cBallControllerMACEDPG::GetActorOutputSize() const
{
	return GetNumActionFrags() * GetActionFragSize();
}

int cBallControllerMACEDPG::GetCriticInputSize() const
{
	return GetStateSize() + GetActionSize();
}

void cBallControllerMACEDPG::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int num_actions = GetNumActionFrags();
	int action_size = GetActionFragSize();
	int output_size = num_actions * action_size;

	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double dist_scale = 2 / (max_dist - min_dist);

	double dist_steps = (max_dist - min_dist) / (num_actions + 1);
	for (int a = 0; a < num_actions; ++a)
	{
		double dist_offset = -(a + 1) * dist_steps;
		out_offset.segment(a * action_size, action_size) *= dist_offset;
		out_scale.segment(a * action_size, action_size) *= dist_scale;
	}
}

void cBallControllerMACEDPG::LoadNetIntern(const std::string& net_file)
{
	cBallControllerDPG::LoadNetIntern(net_file);
	UpdateFragParams();
}

void cBallControllerMACEDPG::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = cMACEDPGTrainer::CalcNumFrags(num_outputs, GetActionFragSize());
	mBoltzmannBuffer.resize(mNumActionFrags);
}

void cBallControllerMACEDPG::CalcCriticVals(const Eigen::VectorXd& state, const Eigen::VectorXd& actions, Eigen::VectorXd& out_vals)
{
	int state_size = GetStateSize();
	int action_size = GetActionSize();
	Eigen::VectorXd state_action = Eigen::VectorXd(state_size + action_size);
	state_action.segment(0, state_size) = state;

	const auto& critic = GetCritic();
	assert(state_action.size() == critic.GetInputSize());

	int num_actions = static_cast<int>(actions.size()) / action_size;
	Eigen::VectorXd critic_output;
	for (int a = 0; a < num_actions; ++a)
	{
		state_action.segment(state_size, action_size) = actions.segment(a * action_size, action_size);
		critic.Eval(state_action, critic_output);
		double val = critic_output[0];
		out_vals[a] = val;
	}
}

void cBallControllerMACEDPG::BuildActorAction(const Eigen::VectorXd& actions, int a_id, tAction& out_action) const
{
	Eigen::VectorXd action_frag;
	cMACEDPGTrainer::GetFrag(actions, GetActionFragSize(), a_id, action_frag);
	out_action.mID = a_id;
	out_action.mDist = action_frag[0];
}

void cBallControllerMACEDPG::UpdateAction()
{
	mExpCritic = false;
	mExpActor = false;
	cBallControllerDPG::UpdateAction();
}

void cBallControllerMACEDPG::DecideAction(tAction& out_action)
{
	DecideActionBoltzmann(out_action);
}

void cBallControllerMACEDPG::DecideActionBoltzmann(tAction& out_action)
{
	mOffPolicy = false;
	const auto& actor = GetActor();

	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd actions;
	actor.Eval(state, actions);
	CalcCriticVals(state, actions, mBoltzmannBuffer);

	int a_max = cMACEDPGTrainer::GetMaxFragValIdx(mBoltzmannBuffer);
	int a = a_max;

	if (mEnableExp && mExpTemp != 0)
	{
		int num_actors = GetNumActionFrags();
		double max_val = cMACEDPGTrainer::GetVal(mBoltzmannBuffer, a_max);

		double sum = 0;
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = cMACEDPGTrainer::GetVal(mBoltzmannBuffer, i);
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

	BuildActorAction(actions, a, out_action);

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

	printf("action: %i, %.5f\n", out_action.mID, out_action.mDist);
}