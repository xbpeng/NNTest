#pragma once
#include "BallControllerCont.h"

class cBallControllerContAC : public cBallControllerCont
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerContAC(cBall& ball);
	virtual ~cBallControllerContAC();

	virtual int GetNetOutputSize() const;

	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual tAction BuildActionFromParams(const Eigen::VectorXd& action_params) const;

protected:

	virtual tAction CalcActionNetCont();
};