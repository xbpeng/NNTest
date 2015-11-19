#pragma once

#include "ArmNNPixelController.h"

class cArmNNPixelNoPoseController : public cArmNNPixelController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNPixelNoPoseController();
	virtual ~cArmNNPixelNoPoseController();

	virtual int GetPoliStateSize() const;
	virtual void BuildPoliStateOffsetScale(Eigen::VectorXd& out_mean, Eigen::VectorXd& out_stdev) const;

protected:
	virtual void UpdatePoliState();
};