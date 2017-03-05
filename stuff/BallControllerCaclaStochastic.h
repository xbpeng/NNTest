#pragma once
#include "BallControllerCacla.h"

class cBallControllerCaclaStochastic : public cBallControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cBallControllerCaclaStochastic(cBall& ball);
	virtual ~cBallControllerCaclaStochastic();

	virtual int GetStateSize() const;
	virtual void BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual int GetActionSize() const;

protected:

	virtual void DecideAction(tAction& out_action);
	
	virtual void GetRandomActionCont(tAction& out_action);
	virtual void ApplyStateExpNoise(Eigen::VectorXd& out_state) const;

	virtual int GetNumNoiseUnits() const;
};