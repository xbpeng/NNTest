#pragma once

#include "ArmControllerMACE.h"
#include "sim/ImpPDController.h"

class cArmPDControllerMACE : public cArmControllerMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmPDControllerMACE();
	virtual ~cArmPDControllerMACE();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	
protected:
	cImpPDController mImpPDCtrl;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;
	virtual void ApplyPoliAction(double time_step, const tAction& action);
};