#pragma once

#include "ArmQPController.h"
#include "sim/ImpPDController.h"

class cArmPDQPController : public cArmQPController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmPDQPController();
	virtual ~cArmPDQPController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

protected:
	cImpPDController mImpPDCtrl;

	virtual void TorquesToTheta(const Eigen::VectorXd& torques, Eigen::VectorXd& out_theta) const;

	virtual void UpdatePoliAction();
	virtual void ApplyPoliAction(double time_step, const tAction& action);
};