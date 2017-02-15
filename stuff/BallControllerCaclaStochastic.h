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

protected:

	double mStateExpNoise;

	virtual void GetRandomActionCont(tAction& out_action);
	virtual void ApplyStateExpNoise(Eigen::VectorXd& out_state) const;
};