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

	virtual void SetTorqueLimit(double torque_lim);
	virtual void SetUpdatePeriod(double period);
	virtual bool NeedUpdate() const;

	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliState(Eigen::VectorXd& out_state) const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	virtual int GetTargetPosSize() const;

	virtual void BuildPoliStateScale(Eigen::VectorXd& out_mean, Eigen::VectorXd& out_stdev) const;

	virtual void SetTargetPos(const tVector& target);

protected:
	static const double gTorqueScale;

	double mTorqueLim;
	double mUpdatePeriod;
	double mUpdateCounter;

	Eigen::VectorXd mPoliState;
	Eigen::VectorXd mPoliAction;
	tVector mTargetPos;
	
	virtual int GetEndEffectorID() const;
	virtual void InitPoliState();
	virtual void InitPoliAction();
	virtual void UpdatePoliState();
	virtual void UpdatePoliAction();
	
	virtual void ApplyPoliAction(double time_step, const Eigen::VectorXd& action);
	virtual void ApplyTorqueLimit(double lim);
};