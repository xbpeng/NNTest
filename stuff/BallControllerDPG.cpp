#include "BallControllerDPG.h"
#include "Ball.h"

cBallControllerDPG::cBallControllerDPG(cBall& ball) :
	cBallControllerCacla(ball)
{
	mExpParams.mNoise = 0.5;
}

cBallControllerDPG::~cBallControllerDPG()
{
}