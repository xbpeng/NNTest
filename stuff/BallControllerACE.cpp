#include "BallControllerACE.h"
#include "Ball.h"

const int gNumActionFrags = 4;
const int gActionFragSize = 1;

int cBallControllerACE::GetMaxFragIdx(const Eigen::VectorXd& params)
{
	return cEACTrainer::GetMaxFragIdx(params, gNumActionFrags, gActionFragSize);
}

double cBallControllerACE::GetMaxFragVal(const Eigen::VectorXd& params)
{
	return cEACTrainer::GetMaxFragVal(params, gNumActionFrags, gActionFragSize);
}

void cBallControllerACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action)
{
	cEACTrainer::GetFrag(params, gNumActionFrags, gActionFragSize, a_idx, out_action);
}

void cBallControllerACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params)
{
	cEACTrainer::SetFrag(frag, a_idx, gNumActionFrags, gActionFragSize, out_params);
}

cBallControllerACE::cBallControllerACE(cBall& ball) :
	cBallController(ball)
{
	mExploring = false;
}

cBallControllerACE::~cBallControllerACE()
{
}

void cBallControllerACE::Reset()
{
	cBallController::Reset();
	mExploring = false;
}

int cBallControllerACE::GetActionSize() const
{
	return gNumActionFrags + gNumActionFrags * gActionFragSize;
}


void cBallControllerACE::ApplyRandAction()
{
	const double noise_prob = 0.5;

	mOffPolicy = true;
	mExploring = false;

	tAction action = GetRandomActionFrag();

	double rand = cMathUtil::RandDouble();
	if (rand < noise_prob)
	{
		mExploring = true;
		AddExpNoise(action);
	}
	
	ApplyAction(action);

	printf("rand action: %i, %.3f\n", action.mID, action.mDist);
}

int cBallControllerACE::GetNumActionFrags() const
{
	return gNumActionFrags;
}

int cBallControllerACE::GetActionFragSize() const
{
	return gActionFragSize;
}

void cBallControllerACE::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());

	int a = mCurrAction.mID;
	out_action[a] = 1;

	Eigen::VectorXd frag = Eigen::VectorXd::Zero(gActionFragSize);
	frag[0] = mCurrAction.mDist;
	SetFrag(frag, a, out_action);
}

cBallControllerACE::tAction cBallControllerACE::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
	tAction action;
	action.mDist = action_params[0];
	return action;
}

bool cBallControllerACE::IsExploring() const
{
	return mExploring;
}

void cBallControllerACE::UpdateAction()
{
	mOffPolicy = false;
	mExploring = false;

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

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	int a = GetMaxFragIdx(action);
	Eigen::VectorXd action_frag;
	GetFrag(action, a, action_frag);

	tAction ball_action;
	ball_action.mID = a;
	ball_action.mDist = action_frag[0];
	printf("action: %i, %.5f\n", a, ball_action.mDist);

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

cBallController::tAction cBallControllerACE::GetRandomActionNoise()
{
	tAction action = CalcActionNetCont();
	AddExpNoise(action);
	return action;
}

void cBallControllerACE::AddExpNoise(tAction& out_action)
{
	const double dist_mean = 0;
	const double dist_stdev = 0.5;

	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);

	out_action.mDist += rand_dist;
	out_action.mDist = cMathUtil::Clamp(out_action.mDist, gMinDist, gMaxDist);
}