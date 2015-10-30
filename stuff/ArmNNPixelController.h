#pragma once

#include "ArmNNController.h"

class cArmNNPixelController : public cArmNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNPixelController();
	virtual ~cArmNNPixelController();

	virtual void SetViewBuffer(const Eigen::VectorXd& view_buff);

protected:
	Eigen::VectorXd mViewBuffer;
	virtual void UpdatePoliState();
};