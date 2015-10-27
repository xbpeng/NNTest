#pragma once

#include "sim/CharController.h"

class cArmController : public cCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmController();
	virtual ~cArmController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual const Eigen::VectorXd& GetControlForce() const;
	virtual void SetTorqueLimit(double torque_lim);
	virtual void SetUpdatePeriod(double period);
	virtual bool NeedUpdate() const;
	virtual int GetControlDim() const;

protected:
	double mTorqueLim;
	double mUpdatePeriod;
	double mUpdateCounter;

	Eigen::VectorXd mCtrlForce;
	
	virtual void UpdateContolForce();
	virtual void InitControlForce();
	
	virtual void ApplyControlForce(const Eigen::VectorXd& tau) const;
};