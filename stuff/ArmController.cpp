#include "ArmController.h"
#include "SimArm.h"

#define ENABLE_MAX_COORD

const int gPosDim = 2;

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
#if defined(ENABLE_MAX_COORD)
	int num_joints = mChar->GetNumJoints();
	int pose_dim = (num_joints - 1) * gPosDim;
	int state_size = target_size + 2 * pose_dim;
#else
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = target_size + 2 * pose_dim;
#endif
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
	return gPosDim;
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
	
#if defined(ENABLE_MAX_COORD)
	for (int j = 1; j < mChar->GetNumJoints(); ++j)
	{
		double chain_len = mChar->CalcJointChainLength(j);
		chain_len = std::max(chain_len, 0.01);
		double pose_scale = 1 / chain_len;
		double vel_scale = 1 / (10 * chain_len);

		int offset = target_size + (j - 1) * gPosDim;
		out_scale.segment(offset, gPosDim) *= pose_scale;
		out_scale.segment(offset + pose_size, gPosDim) *= vel_scale;
	}
#else
	double pose_scale = (1 / M_PI);
	double vel_scale = (1 / (10 * M_PI));
	out_scale.segment(target_size, pose_size) = pose_scale * Eigen::VectorXd::Ones(pose_size);
	out_scale.segment(target_size + pose_size, pose_size) = vel_scale * Eigen::VectorXd::Ones(pose_size);
#endif
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
	return static_cast<int>(mChar->GetNumJoints() - 1);
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

	int poli_state_size = static_cast<int>(mPoliState.size());
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int idx_beg = root_size;
	int target_size = GetTargetPosSize();
	int param_size = (poli_state_size - target_size) / 2;
	int num_joints = mChar->GetNumJoints();

	tVector root_pos = mChar->GetRootPos();
	mPoliState.segment(0, target_size) = (mTargetPos - root_pos).segment(0, target_size);
	
#if defined(ENABLE_MAX_COORD)
	for (int j = 1; j < num_joints; ++j)
	{
		tVector joint_pos = mChar->CalcJointPos(j);
		tVector joint_vel = mChar->CalcJointVel(j);
		joint_pos -= root_pos;
		int offset = target_size + (j - 1) * gPosDim;
		mPoliState.segment(offset, gPosDim) = joint_pos.segment(0, gPosDim);
		mPoliState.segment(offset + param_size, gPosDim) = joint_vel.segment(0, gPosDim);
	}
#else
	mPoliState.segment(target_size, param_size) = pose.segment(root_size, param_size);
	mPoliState.segment(target_size + param_size, param_size) = vel.segment(root_size, param_size);
#endif
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

	int num_joints = mChar->GetNumJoints();
	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			double t = action[idx];
			t = cMathUtil::Clamp(t, -mTorqueLim, mTorqueLim);
			tVector torque = tVector(0, 0, t, 0);
			joint.AddTorque(torque);
			++idx;
		}
	}
	assert(idx == action.size());
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
