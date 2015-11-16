#pragma once
#include "BallController.h"

class cBallControllerCont : public cBallController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static const double gMinDist;
	static const double gMaxDist;

	cBallControllerCont(cBall& ball);
	virtual ~cBallControllerCont();

	virtual int GetActionSize() const;
	virtual void ApplyRandAction();

	virtual void RecordAction(Eigen::VectorXd& out_action) const;

protected:

	virtual void UpdateAction();
	virtual tAction CalcActionNetCont();
	virtual tAction GetRandomActionCont();
};