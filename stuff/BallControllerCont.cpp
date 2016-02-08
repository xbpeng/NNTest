#include "BallControllerCont.h"
#include "Ball.h"

cBallControllerCont::cBallControllerCont(cBall& ball) :
	cBallController(ball)
{
	mExpNoiseMean = 0;
	mExpNoiseStd = 0.5;
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
	tAction action;
	GetRandomActionCont(action);
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

void cBallControllerCont::CalcActionNet(tAction& out_action)
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	tAction ball_action;
	ball_action.mDist = action[0];
	printf("action: %.5f , %.5f\n", action[0], ball_action.mDist);

	out_action = ball_action;
}

void cBallControllerCont::GetRandomAction(tAction& out_action)
{
	GetRandomActionCont(out_action);
}

void cBallControllerCont::GetRandomActionCont(tAction& out_action)
{
	tAction action;
	CalcActionNet(action);
	double rand_dist = cMathUtil::RandDoubleNorm(mExpNoiseMean, mExpNoiseStd);
	
	action.mDist += rand_dist;
	action.mDist = cMathUtil::Clamp(action.mDist, gMinDist, gMaxDist);

	out_action = action;
}