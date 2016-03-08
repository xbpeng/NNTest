#pragma once

#include "ArmNNController.h"
#include "sim/ImpPDController.h"

class cArmVelNNController : public cArmNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmVelNNController();
	virtual ~cArmVelNNController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

protected:
	cImpPDController mImpPDCtrl;

	virtual void UpdatePoliAction();
	virtual void ApplyPoliAction(double time_step, const Eigen::VectorXd& action);
};