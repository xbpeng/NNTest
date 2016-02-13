#pragma once
#include "BallControllerCacla.h"

class cBallControllerDPG : public cBallControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerDPG(cBall& ball);
	virtual ~cBallControllerDPG();

protected:
};