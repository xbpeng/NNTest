#include "BallControllerCaclaStochastic.h"

const int gExpNoiseSize = 64;

cBallControllerCaclaStochastic::cBallControllerCaclaStochastic(cBall& ball) :
								cBallControllerCacla(ball)
{
}

cBallControllerCaclaStochastic::~cBallControllerCaclaStochastic()
{
}

int cBallControllerCaclaStochastic::GetStateSize() const
{
	int state_size = cBallControllerCacla::GetStateSize();
	state_size += gExpNoiseSize;
	return state_size;
}

void cBallControllerCaclaStochastic::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cBallControllerCacla::BuildNNOutputOffsetScale(out_offset, out_scale);
	int exp_offset = cBallControllerCacla::GetStateSize();
	int exp_size = gExpNoiseSize;
	out_offset.segment(exp_offset, exp_size).setZero();
	out_scale.segment(exp_offset, exp_size).setOnes();
}

void cBallControllerCaclaStochastic::GetRandomActionCont(tAction& out_action)
{
	ApplyStateExpNoise(mPoliState);
	cBallControllerCacla::GetRandomActionCont(out_action);
}

void cBallControllerCaclaStochastic::ApplyStateExpNoise(Eigen::VectorXd& out_state) const
{
	int exp_offset = cBallControllerCacla::GetStateSize();
	double exp_scale = 0.1;
	for (int i = 0; i < gExpNoiseSize; ++i)
	{
		double curr_noise = cMathUtil::RandDouble(0, exp_scale);
		out_state[exp_offset + i] = curr_noise;
	}
}