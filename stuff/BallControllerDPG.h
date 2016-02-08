#pragma once
#include "BallControllerCont.h"

class cBallControllerDPG : public cBallControllerCont
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerDPG(cBall& ball);
	virtual ~cBallControllerDPG();

protected:
};