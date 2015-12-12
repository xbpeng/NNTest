#include "BallControllerCont.h"
#include "Ball.h"

cBallControllerCont::cBallControllerCont(cBall& ball) :
	cBallController(ball)
{
}

cBallControllerCont::~cBallControllerCont()
{
}

int cBallControllerCont::GetActionSize() const
{
	return 1;
}

void cBallControllerCont::ApplyRandAction()
{
	tAction action = GetRandomActionCont();
	ApplyAction(action);
	mOffPolicy = true;
	printf("rand action: %.3f\n", action.mDist);
}

void cBallControllerCont::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());
	out_action[0] = mCurrAction.mDist;
}

cBallControllerCont::tAction cBallControllerCont::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == 1);
	tAction action;
	action.mDist = action_params[0];
	return action;
}

void cBallControllerCont::UpdateAction()
{
	mOffPolicy = false;
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

cBallController::tAction cBallControllerCont::CalcActionNetCont()
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	tAction ball_action;
	ball_action.mDist = action[0];
	printf("action: %.5f , %.5f\n", action[0], ball_action.mDist);

	return ball_action;
}

cBallController::tAction cBallControllerCont::GetRandomActionCont()
{
	const double dist_mean = 0;
	const double dist_stdev = 0.5;

	tAction action = CalcActionNetCont();
	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	/*
	while (action.mDist + rand_dist <= gMinDist
		|| action.mDist + rand_dist >= gMaxDist)
	{
		rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	}
	*/
	action.mDist += rand_dist;
	action.mDist = cMathUtil::Clamp(action.mDist, gMinDist, gMaxDist);

	//tAction action = GetRandomActionDiscrete();
	return action;
}