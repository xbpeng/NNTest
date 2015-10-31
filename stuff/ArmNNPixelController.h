#pragma once

#include "ArmNNController.h"

class cArmNNPixelController : public cArmNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNPixelController();
	virtual ~cArmNNPixelController();

	virtual int GetPoliStateSize() const;
	virtual void BuildPoliStateScale(Eigen::VectorXd& out_mean, Eigen::VectorXd& out_stdev) const;

	virtual void SetViewBuffer(const Eigen::VectorXd& view_buff);
	
protected:
	Eigen::VectorXd mViewBuffer;
	virtual void UpdatePoliState();
};