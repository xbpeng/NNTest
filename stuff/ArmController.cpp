#include "ArmController.h"
#include "SimArm.h"

const double cArmController::gTorqueScale = 0.1;

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
	InitPoliState();
	InitPoliAction();
}

void cArmController::Reset()
{
	cCharController::Reset();
	mUpdateCounter = mUpdatePeriod;
	mPoliAction.setZero();
	mPoliState.setZero();
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
		UpdatePoliState();
		UpdatePoliAction();
		mUpdateCounter = 0;
	}

	ApplyPoliAction(mPoliAction);

	mUpdateCounter += time_step;
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

int cArmController::GetPoliStateSize() const
{
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = 2 * pose_dim;
	return state_size;
}

int cArmController::GetPoliActionSize() const
{
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int action_dim = num_dof - root_size;
	return action_dim;
}

void cArmController::RecordPoliState(Eigen::VectorXd& out_state) const
{
	out_state = mPoliState;
}

void cArmController::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	out_action = mPoliAction;
}

void cArmController::InitPoliState()
{
	mPoliState= Eigen::VectorXd::Zero(GetPoliStateSize());
}

void cArmController::InitPoliAction()
{
	mPoliAction = Eigen::VectorXd::Zero(GetPoliActionSize());
}

void cArmController::UpdatePoliState()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int idx_beg = root_size;
	int param_size = static_cast<int>(pose.size()) - root_size;

	mPoliState.segment(0, param_size) = pose.segment(root_size, param_size);
	mPoliState.segment(param_size, param_size) = vel.segment(root_size, param_size);
}

void cArmController::UpdatePoliAction()
{
}

void cArmController::ApplyPoliAction(const Eigen::VectorXd& action) const
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.size()) == num_joints - 1);

	for (int j = 1; j < num_joints; ++j)
	{
		cJoint& joint = mChar->GetJoint(j);
		double t = action[j - 1];
		t /= gTorqueScale;
		t = std::min(mTorqueLim, t);
		tVector torque = tVector(0, 0, t, 0);
		joint.AddTorque(torque);
	}
}