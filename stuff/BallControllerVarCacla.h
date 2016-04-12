#pragma once
#include "BallControllerCacla.h"

class cBallControllerVarCacla : public cBallControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerVarCacla(cBall& ball);
	virtual ~cBallControllerVarCacla();

	virtual int GetActorOutputSize() const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:

	virtual void GetRandomActionCont(tAction& out_action);
};