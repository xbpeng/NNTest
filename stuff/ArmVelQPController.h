#pragma once

#include "ArmQPController.h"
#include "sim/ImpPDController.h"

class cArmVelQPController : public cArmQPController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cArmVelQPController();
	virtual ~cArmVelQPController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();

protected:
	cImpPDController mImpPDCtrl;

	virtual void TorquesToVel(const Eigen::VectorXd& torques, Eigen::VectorXd& out_vel) const;

	virtual void UpdatePoliAction();
	virtual void ApplyPoliAction(double time_step, const tAction& action);
};