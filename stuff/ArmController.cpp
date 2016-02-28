#include "ArmController.h"
#include "SimArm.h"

cArmController::cArmController()
{
	mTorqueLim = 300;
	mUpdatePeriod = 1 / 60.0;
	mUpdateCounter = mUpdatePeriod;
	mTargetPos = tVector::Zero();
}

cArmController::~cArmController()
{
}

void cArmController::Init(cSimCharacter* character)
{
	cCharController::Init(character);
	mTargetPos = mChar->CalcJointPos(GetEndEffectorID());

	InitPoliState();
	InitPoliAction();
	ApplyTorqueLimit(mTorqueLim);
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
		DecideAction();
		mUpdateCounter = 0;
	}

	ApplyPoliAction(time_step, mPoliAction);
	mUpdateCounter += time_step;
}

void cArmController::SetTorqueLimit(double torque_lim)
{
	mTorqueLim = torque_lim;
	ApplyTorqueLimit(torque_lim);
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
	int target_size = GetTargetPosSize();
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = target_size + 2 * pose_dim;
	return state_size;
}

int cArmController::GetPoliActionSize() const
{
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int action_dim = num_dof - root_size - 1; // -1 for end effector
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

int cArmController::GetTargetPosSize() const
{
	return 2;
}

void cArmController::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double pos_scale = 2;
	int state_size = GetPoliStateSize();

	out_offset = Eigen::VectorXd::Zero(state_size);
	out_scale = Eigen::VectorXd::Ones(state_size);
	int target_size = GetTargetPosSize();
	int pose_size = (state_size - target_size) / 2;

	out_scale.segment(0, target_size) = (1 / pos_scale) * Eigen::VectorXd::Ones(target_size);
	out_scale.segment(target_size, pose_size) = (1 / M_PI) * Eigen::VectorXd::Ones(pose_size);
	out_scale.segment(target_size + pose_size, pose_size) = (1 / (2 * M_PI)) * Eigen::VectorXd::Ones(pose_size);
}

void cArmController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double torque_scale = 0.01;
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = torque_scale * Eigen::VectorXd::Ones(output_size);
}

void cArmController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int output_size = GetPoliActionSize();
	out_min = -mTorqueLim * Eigen::VectorXd::Ones(output_size);
	out_max = mTorqueLim * Eigen::VectorXd::Ones(output_size);
}

void cArmController::SetTargetPos(const tVector& target)
{
	mTargetPos = target;
}

int cArmController::GetEndEffectorID() const
{
	return static_cast<int>(cSimArm::eJointLinkEnd);
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
	int target_size = GetTargetPosSize();

	mPoliState.segment(0, target_size) = mTargetPos.segment(0, target_size);
	mPoliState.segment(target_size, param_size) = pose.segment(root_size, param_size);
	mPoliState.segment(target_size + param_size, param_size) = vel.segment(root_size, param_size);
}

void cArmController::UpdatePoliAction()
{
}

void cArmController::DecideAction()
{
	UpdatePoliAction();
}

void cArmController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	assert(static_cast<int>(action.size()) == GetPoliActionSize());

	int joint_offset = 1;
	for (int j = 0; j < action.size(); ++j)
	{
		cJoint& joint = mChar->GetJoint(j + joint_offset);
		if (joint.IsValid())
		{
			double t = action[j];
			t = cMathUtil::Clamp(t, -mTorqueLim, mTorqueLim);
			tVector torque = tVector(0, 0, t, 0);
			joint.AddTorque(torque);
		}
	}
}

void cArmController::ApplyTorqueLimit(double lim)
{
	int num_joints = mChar->GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = mChar->GetJoint(j);
		joint.SetTorqueLimit(lim);
	}
}
