#include "BallControllerDPG.h"
#include "Ball.h"

cBallControllerDPG::cBallControllerDPG(cBall& ball) :
	cBallControllerCacla(ball)
{
	mExpNoiseStd = 0.5;
}

cBallControllerDPG::~cBallControllerDPG()
{
}