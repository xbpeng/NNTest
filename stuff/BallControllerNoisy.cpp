#include "BallControllerNoisy.h"
#include "Ball.h"

cBallControllerNoisy::cBallControllerNoisy(cBall& ball) :
	cBallControllerCacla(ball)
{
	mExpNoiseStd = 0;
}

cBallControllerNoisy::~cBallControllerNoisy()
{
}

int cBallControllerNoisy::GetActionSize() const
{
	int size = cBallControllerCacla::GetActionSize();
	size += GetNumNoisyUnits();
	return size;
}

int cBallControllerNoisy::GetNumNoisyUnits() const
{
	int num = 0;
	if (mNet.HasNet())
	{
		int output_size = mNet.GetOutputSize();
		int action_size = cBallControllerCacla::GetActionSize();
		num = output_size - action_size;
	}
	return num;
}