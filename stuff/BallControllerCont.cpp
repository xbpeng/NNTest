#include "BallControllerCont.h"
#include "Ball.h"

const double gMinDist = 0.1;

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
	printf("rand action: %.3f\n", action.mDist);
}

void cBallControllerCont::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());
	out_action[0] = mCurrAction.mDist;
}

void cBallControllerCont::UpdateAction()
{
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
	ball_action.mDist = std::max(gMinDist, action[0]);
	printf("action: %.5f \n", action[0]);

	return ball_action;
}

cBallController::tAction cBallControllerCont::GetRandomActionCont()
{
	const double dist_mean = 0;
	const double dist_stdev = 0.2;

	tAction action = CalcActionNetCont();
	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	while (action.mDist + rand_dist <= gMinDist)
	{
		rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	}
	action.mDist += rand_dist;
	return action;
}