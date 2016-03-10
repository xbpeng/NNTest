#pragma once

#include "sim/CharController.h"

class cArmController : public cCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tAction
	{
		int mID;
		Eigen::VectorXd mParams;
	};

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

	virtual void BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;
	virtual void SetTargetPos(const tVector& target);

	virtual int GetEndEffectorID() const;
	virtual int GetCurrActionID() const;

protected:
	double mTorqueLim;
	double mUpdatePeriod;
	double mUpdateCounter;

	Eigen::VectorXd mPoliState;
	tAction mPoliAction;
	tVector mTargetPos;
	
	virtual void InitPoliState();
	virtual void InitPoliAction();
	virtual void UpdatePoliState();
	virtual void UpdatePoliAction();
	virtual void DecideAction();

	virtual void ApplyPoliAction(double time_step, const tAction& action);
	virtual void ApplyTorqueLimit(double lim);
};