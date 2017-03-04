#pragma once
#include "BallControllerCacla.h"

class cBallControllerNoisy : public cBallControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerNoisy(cBall& ball);
	virtual ~cBallControllerNoisy();

	virtual int GetActionSize() const;
	
protected:
	
	virtual int GetNumNoisyUnits() const;
};