#include "BallControllerCaclaStochastic.h"

const int gExpNoiseSize = 16;

cBallControllerCaclaStochastic::cBallControllerCaclaStochastic(cBall& ball) :
								cBallControllerCacla(ball)
{
	mExpParams.mNoise = 0.2;
	mExpParams.mInternNoise = 1;
	//mExpParams.mInternNoise = 0;
}

cBallControllerCaclaStochastic::~cBallControllerCaclaStochastic()
{
}

int cBallControllerCaclaStochastic::GetStateSize() const
{
	int state_size = cBallControllerCacla::GetStateSize();
	state_size += GetNumNoiseUnits();
	return state_size;
}

void cBallControllerCaclaStochastic::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	cBallControllerCacla::BuildNNInputOffsetScaleTypes(out_types);
	int offset = cBallControllerCacla::GetStateSize();
	for (int i = 0; i < GetNumNoiseUnits(); ++i)
	{
		out_types[offset + i] = cNeuralNet::eOffsetScaleTypeFixed;
	}
}

void cBallControllerCaclaStochastic::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cBallControllerCacla::BuildActorOutputOffsetScale(out_offset, out_scale);
	
	int offset = cBallControllerCacla::GetActionSize();
	int size = GetNumNoiseUnits();
	out_offset.segment(offset, size) = Eigen::VectorXd::Zero(size);
	out_scale.segment(offset, size) = Eigen::VectorXd::Ones(size);
}

void cBallControllerCaclaStochastic::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	//const double noise_bound = 3;
	const double noise_bound = std::numeric_limits<double>::infinity();

	cBallControllerCacla::BuildActionBounds(out_min, out_max);
	
	int offset = cBallControllerCacla::GetActionSize();
	int size = GetNumNoiseUnits();
	out_min.segment(offset, size) = -noise_bound * Eigen::VectorXd::Ones(size);
	out_max.segment(offset, size) = noise_bound * Eigen::VectorXd::Ones(size);
}

int cBallControllerCaclaStochastic::GetActionSize() const
{
	int size = cBallControllerCacla::GetActionSize();
	size += GetNumNoiseUnits();
	return size;
}

void cBallControllerCaclaStochastic::DecideAction(tAction& out_action)
{
	int exp_offset = cBallControllerCacla::GetStateSize();
	int exp_size = GetNumNoiseUnits();
	mPoliState.segment(exp_offset, exp_size).setZero();

	cBallControllerCacla::DecideAction(out_action);
}

void cBallControllerCaclaStochastic::GetRandomActionCont(tAction& out_action)
{
	ApplyStateExpNoise(mPoliState);
	cBallControllerCacla::GetRandomActionCont(out_action);
}

void cBallControllerCaclaStochastic::ApplyStateExpNoise(Eigen::VectorXd& out_state) const
{
	const double noise_bound = 3 * mExpParams.mInternNoise;

	int exp_offset = cBallControllerCacla::GetStateSize();
	for (int i = 0; i < GetNumNoiseUnits(); ++i)
	{
		double curr_noise = 0;
		do
		{
			curr_noise = cMathUtil::RandDoubleNorm(0, mExpParams.mInternNoise);
		} while (std::abs(curr_noise) > noise_bound);
		
		out_state[exp_offset + i] = curr_noise;
	}
}

int cBallControllerCaclaStochastic::GetNumNoiseUnits() const
{
	return gExpNoiseSize;
}