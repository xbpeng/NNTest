#include "BallControllerDPG.h"
#include "Ball.h"

cBallControllerDPG::cBallControllerDPG(cBall& ball) :
	cBallControllerCont(ball)
{
	mExpNoiseMean = 0;
	mExpNoiseStd = 0.5;
}

cBallControllerDPG::~cBallControllerDPG()
{
}