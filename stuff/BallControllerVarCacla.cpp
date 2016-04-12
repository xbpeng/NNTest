#include "BallControllerVarCacla.h"
#include "Ball.h"

cBallControllerVarCacla::cBallControllerVarCacla(cBall& ball) :
	cBallControllerCacla(ball)
{
}

cBallControllerVarCacla::~cBallControllerVarCacla()
{
}

int cBallControllerVarCacla::GetActorOutputSize() const
{
	return 2 * GetActionSize();
}

void cBallControllerVarCacla::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetActionSize();
	int output_size = GetActorOutputSize();

	double min_dist = cBallControllerVarCacla::gMinDist;
	double max_dist = cBallControllerVarCacla::gMaxDist;
	double offset = -0.5 * (max_dist + min_dist);
	double scale = 2 / (max_dist - min_dist);

	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	out_offset.segment(0, action_size) *= offset;
	out_scale.segment(0, action_size) *= scale;
	out_offset.segment(action_size, action_size) *= 0;
	out_scale.segment(action_size, action_size) *= 1 / mExpNoiseStd;
}

void cBallControllerVarCacla::GetRandomActionCont(tAction& out_action)
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	out_action.mDist = action[0];
	
	double stdev = action[1];
	printf("Mean: %.5f, Stdev: %.5f\n", action[0], action[1]);

	double rand_dist = cMathUtil::RandDoubleNorm(0, stdev);
	out_action.mDist += rand_dist;
}