#include "ArmController.h"
#include "SimArm.h"

cArmController::cArmController()
{
	mTorqueLim = 400;
	mUpdatePeriod = 1 / 60.0;
	mUpdateCounter = mUpdatePeriod;
}

cArmController::~cArmController()
{
}

void cArmController::Init(cSimCharacter* character)
{
	cCharController::Init(character);
	InitControlForce();
}

void cArmController::Reset()
{
	cCharController::Reset();
	mUpdateCounter = mUpdatePeriod;
}

void cArmController::Clear()
{
	cCharController::Clear();
}

void cArmController::Update(double time_step)
{
	cCharController::Update(time_step);

	if (NeedUpdate())
	{
		UpdateContolForce();
		mUpdateCounter = 0;
	}

	Eigen::VectorXd tau = GetControlForce();
	ApplyControlForce(tau);

	mUpdateCounter += time_step;
}

const Eigen::VectorXd& cArmController::GetControlForce() const
{
	return mCtrlForce;
}

void cArmController::SetTorqueLimit(double torque_lim)
{
	mTorqueLim = torque_lim;
}

void cArmController::SetUpdatePeriod(double period)
{
	mUpdatePeriod = period;
}

bool cArmController::NeedUpdate() const
{
	return mUpdateCounter >= mUpdatePeriod;
}

int cArmController::GetControlDim() const
{
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int ctrl_dim = num_dof - root_size;
	return ctrl_dim;
}

void cArmController::UpdateContolForce()
{
}

void cArmController::InitControlForce()
{
	mCtrlForce = Eigen::VectorXd::Zero(GetControlDim());
}

void cArmController::ApplyControlForce(const Eigen::VectorXd& ctrl_force) const
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(ctrl_force.size()) == num_joints - 1);

	for (int j = 1; j < num_joints; ++j)
	{
		cJoint& joint = mChar->GetJoint(j);
		tVector torque = tVector(0, 0, ctrl_force[j - 1], 0);
		joint.AddTorque(torque);
	}
}