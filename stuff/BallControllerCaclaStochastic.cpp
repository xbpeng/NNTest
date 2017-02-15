#include "BallControllerCaclaStochastic.h"

const int gExpNoiseSize = 64;

cBallControllerCaclaStochastic::cBallControllerCaclaStochastic(cBall& ball) :
								cBallControllerCacla(ball)
{
	mExpNoiseStd = 0.5;
	//mStateExpNoise = 0.2;
	mStateExpNoise = 0;
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

void cBallControllerCaclaStochastic::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cBallControllerCacla::BuildNNInputOffsetScaleTypes(out_types);
	int offset = cBallControllerCacla::GetStateSize();
	for (int i = 0; i < gExpNoiseSize; ++i)
	{
		out_types[offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

void cBallControllerCaclaStochastic::GetRandomActionCont(tAction& out_action)
{
	ApplyStateExpNoise(mPoliState);
	cBallControllerCacla::GetRandomActionCont(out_action);
}

void cBallControllerCaclaStochastic::ApplyStateExpNoise(Eigen::VectorXd& out_state) const
{
	int exp_offset = cBallControllerCacla::GetStateSize();
	for (int i = 0; i < gExpNoiseSize; ++i)
	{
		double curr_noise = cMathUtil::RandDouble(0, mStateExpNoise);
		out_state[exp_offset + i] = curr_noise;
	}
}