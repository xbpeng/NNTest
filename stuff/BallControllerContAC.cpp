#include "BallControllerContAC.h"
#include "Ball.h"

const int gParamValIdx = 0;
const int gParamActionIdx = 1;

cBallControllerContAC::cBallControllerContAC(cBall& ball) :
	cBallControllerCont(ball)
{
}

cBallControllerContAC::~cBallControllerContAC()
{
}

int cBallControllerContAC::GetNetOutputSize() const
{
	return 1 + GetActionSize(); // + 1 for crtic value
}

void cBallControllerContAC::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());
	out_action[0] = mCurrAction.mDist;
}

cBallControllerContAC::tAction cBallControllerContAC::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
	tAction action;
	action.mDist = action_params[0];
	return action;
}

cBallController::tAction cBallControllerContAC::CalcActionNetCont()
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd y;
	mNet.Eval(state, y);

	tAction ball_action;
	ball_action.mDist = y[gParamActionIdx];
	printf("action: %.5f , value %.5f\n", y[gParamActionIdx], y[gParamValIdx]);

	return ball_action;
}