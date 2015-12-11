#include "BallControllerQAC.h"
#include "Ball.h"

const int gNumActionFrags = 2;
const int gActionFragSize = 1;

int cBallControllerQAC::GetMaxFragIdx(const Eigen::VectorXd& params)
{
	return cQACTrainer::GetMaxFragIdx(params, gNumActionFrags, gActionFragSize);
}

double cBallControllerQAC::GetMaxFragVal(const Eigen::VectorXd& params)
{
	return cQACTrainer::GetMaxFragVal(params, gNumActionFrags, gActionFragSize);
}

void cBallControllerQAC::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action)
{
	cQACTrainer::GetFrag(params, gNumActionFrags, gActionFragSize, a_idx, out_action);
}

void cBallControllerQAC::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params)
{
	cQACTrainer::SetFrag(frag, a_idx, gNumActionFrags, gActionFragSize, out_params);
}

cBallControllerQAC::cBallControllerQAC(cBall& ball) :
	cBallController(ball)
{
	mExploring = false;
}

cBallControllerQAC::~cBallControllerQAC()
{
}

void cBallControllerQAC::Reset()
{
	mExploring = false;
}

int cBallControllerQAC::GetActionSize() const
{
	return gNumActionFrags + gNumActionFrags * gActionFragSize;
}


void cBallControllerQAC::ApplyRandAction()
{
	tAction action;
	
	double explore_prob = 0.5;
	double rand = cMathUtil::RandDouble();
	if (rand < explore_prob)
	{
		mExploring = true;
		action = GetRandomActionExplore();
	}
	else
	{
		action = GetRandomActionCont();
	}
	
	ApplyAction(action);
	mOffPolicy = true;

	printf("rand action: %.3f\n", action.mDist);
}

int cBallControllerQAC::GetNumActionFrags() const
{
	return gNumActionFrags;
}

int cBallControllerQAC::GetActionFragSize() const
{
	return gActionFragSize;
}

void cBallControllerQAC::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());

	int a = mCurrAction.mID;
	out_action[a] = 1;

	Eigen::VectorXd frag = Eigen::VectorXd::Zero(gActionFragSize);
	frag[0] = mCurrAction.mDist;
	SetFrag(frag, a, out_action);
}

cBallControllerQAC::tAction cBallControllerQAC::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
	tAction action;
	action.mDist = action_params[0];
	return action;
}

bool cBallControllerQAC::IsExploring() const
{
	return mExploring;
}

void cBallControllerQAC::UpdateAction()
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
		action = GetRandomActionCont();
		mOffPolicy = true;
	}

	ApplyAction(action);
}

cBallController::tAction cBallControllerQAC::CalcActionNetCont()
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

cBallController::tAction cBallControllerQAC::GetRandomActionCont()
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

cBallController::tAction cBallControllerQAC::GetRandomActionExplore()
{
	const double dist_mean = 0;
	const double dist_stdev = 0.5;

	tAction action = CalcActionNetCont();
	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	
	action.mDist += rand_dist;
	action.mDist = cMathUtil::Clamp(action.mDist, gMinDist, gMaxDist);

	return action;
}