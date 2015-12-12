#include "BallControllerACE.h"
#include "Ball.h"

const int gActionFragSize = 1;

cBallControllerACE::cBallControllerACE(cBall& ball) :
	cBallController(ball)
{
	mNumActionFrags = 0;

	mExpCritic = false;
	mExpActor = false;
}

cBallControllerACE::~cBallControllerACE()
{
}

void cBallControllerACE::Reset()
{
	cBallController::Reset();
	mExpCritic = false;
	mExpActor = false;
}

bool cBallControllerACE::LoadNet(const std::string& net_file)
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

int cBallControllerACE::GetActionSize() const
{
	return mNumActionFrags + mNumActionFrags * gActionFragSize;
}


void cBallControllerACE::ApplyRandAction()
{
	const double noise_prob = 0.5;

	mOffPolicy = true;
	mExpCritic = true;
	mExpActor = false;

	tAction action = GetRandomActionFrag();

	double rand = cMathUtil::RandDouble();
	if (rand < noise_prob)
	{
		mExpActor = true;
		AddExpNoise(action);
	}
	
	ApplyAction(action);

	printf("rand action: %i, %.3f\n", action.mID, action.mDist);
}

int cBallControllerACE::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBallControllerACE::GetActionFragSize() const
{
	return gActionFragSize;
}

void cBallControllerACE::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());

	int a = mCurrAction.mID;
	SetTupleVal(1, a, out_action);

	Eigen::VectorXd frag = Eigen::VectorXd::Zero(gActionFragSize);
	frag[0] = mCurrAction.mDist;
	SetTupleFrag(frag, a, out_action);
}

cBallControllerACE::tAction cBallControllerACE::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
	int a = GetTupleMaxFragIdx(action_params);
	Eigen::VectorXd action_frag;
	GetTupleFrag(action_params, a, action_frag);

	assert(action_frag.size() == 1);

	tAction action;
	action.mID = a;
	action.mDist = action_frag[0];

	return action;
}

bool cBallControllerACE::IsExpCritic() const
{
	return mExpCritic;
}

bool cBallControllerACE::IsExpActor() const
{
	return mExpActor;
}

void cBallControllerACE::UpdateAction()
{
	mOffPolicy = false;
	mExpCritic = false;
	mExpActor = false;

	mPosBeg = mBall.GetPos();

	if (HasGround())
	{
		SampleGround(mGroundSamples);
	}

	tAction action;
	action.mDist = 0;
	if (HasNet())
	{
		action = CalcActionNetCont();
	}
	else
	{
		action = GetRandomActionFrag();
		mOffPolicy = true;
	}

	ApplyAction(action);
}

cBallController::tAction cBallControllerACE::CalcActionNetCont()
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd y;
	mNet.Eval(state, y);

	tAction ball_action = BuildActionFromParams(y);
	printf("action: %i, %.5f\n", ball_action.mID, ball_action.mDist);

	return ball_action;
}

cBallController::tAction cBallControllerACE::GetRandomActionFrag()
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	int a = cMathUtil::RandInt(0, GetNumActionFrags());
	Eigen::VectorXd action_frag;
	GetFrag(action, a, action_frag);

	tAction ball_action;
	ball_action.mID = a;
	ball_action.mDist = action_frag[0];

	return ball_action;
}

void cBallControllerACE::AddExpNoise(tAction& out_action)
{
	const double dist_mean = 0;
	const double dist_stdev = 0.5;

	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);

	out_action.mDist += rand_dist;
	out_action.mDist = cMathUtil::Clamp(out_action.mDist, gMinDist, gMaxDist);
}

void cBallControllerACE::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = cACETrainer::CalcNumFrags(num_outputs, gActionFragSize);
}

int cBallControllerACE::GetMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

double cBallControllerACE::GetMaxFragVal(const Eigen::VectorXd& params) const
{
	return cACETrainer::GetMaxFragVal(params, mNumActionFrags);
}

void cBallControllerACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cACETrainer::GetFrag(params, mNumActionFrags, gActionFragSize, a_idx, out_action);
}

void cBallControllerACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cACETrainer::SetFrag(frag, a_idx, mNumActionFrags, gActionFragSize, out_params);
}

void cBallControllerACE::SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const
{
	cACETrainer::SetVal(val, a_idx, out_params);
}

int cBallControllerACE::GetTupleMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cACETrainer::GetMaxFragIdx(params, mNumActionFrags);
}

void cBallControllerACE::GetTupleFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cACETrainer::GetFrag(params, mNumActionFrags, gActionFragSize, a_idx, out_action);
}

void cBallControllerACE::SetTupleFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cACETrainer::SetFrag(frag, a_idx, mNumActionFrags, gActionFragSize, out_params);
}

void cBallControllerACE::SetTupleVal(double val, int a_idx, Eigen::VectorXd& out_params) const
{
	cACETrainer::SetVal(val, a_idx, out_params);
}