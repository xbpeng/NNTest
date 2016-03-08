#pragma once

#include "ArmNNPixelController.h"
#include "sim/ImpPDController.h"

class cArmPDNNPixelController : public cArmNNPixelController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmPDNNPixelController();
	virtual ~cArmPDNNPixelController();

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