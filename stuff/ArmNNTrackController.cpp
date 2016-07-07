#include "ArmNNTrackController.h"
#include "sim/SimCharacter.h"

cArmNNTrackController::cArmNNTrackController()
{
}

cArmNNTrackController::~cArmNNTrackController()
{
}

void cArmNNTrackController::Init(cSimCharacter* character)
{
	cArmNNController::Init(character);
	InitTargetPoseVel();
}

void cArmNNTrackController::Reset()
{
	cArmNNController::Reset();
	InitTargetPoseVel();
}

int cArmNNTrackController::GetPoliStateSize() const
{
	int num_joints = mChar->GetNumJoints();
	int pose_dim = (num_joints - 1) * gPosDim;
	int state_size = 4 * pose_dim;
	return state_size;
}

void cArmNNTrackController::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int state_size = GetPoliStateSize();

	out_offset = Eigen::VectorXd::Zero(state_size);
	out_scale = Eigen::VectorXd::Ones(state_size);
	int target_size = GetTargetPosSize();
	int pose_size = state_size / 4;

	for (int j = 1; j < mChar->GetNumJoints(); ++j)
	{
		double chain_len = mChar->CalcJointChainLength(j);
		chain_len = std::max(chain_len, 0.01);
		double pose_scale = 1 / chain_len;
		double vel_scale = 1 / (10 * chain_len);

		int offset = (j - 1) * gPosDim;
		out_scale.segment(offset, gPosDim) *= pose_scale;
		out_scale.segment(offset + pose_size, gPosDim) *= vel_scale;
	}
	out_scale.segment(pose_size * 2, pose_size * 2) = out_scale.segment(0, pose_size * 2);
}

void cArmNNTrackController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);
}

void cArmNNTrackController::SetTargetPoseVel(const Eigen::VectorXd& tar_pose, const Eigen::VectorXd& tar_vel)
{
	assert(tar_pose.size() == mChar->GetNumDof());
	assert(tar_vel.size() == mChar->GetNumDof());
	mTargetPose = tar_pose;
	mTargetVel = tar_vel;
}

void cArmNNTrackController::InitTargetPoseVel()
{
	mTargetPose = mChar->GetPose();
	mTargetVel = mChar->GetPose();
}

void cArmNNTrackController::UpdatePoliState()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	int poli_state_size = static_cast<int>(mPoliState.size());
	int pose_size = (poli_state_size) / 4;
	int num_joints = mChar->GetNumJoints();

	tVector root_pos = mChar->GetRootPos();
	for (int j = 1; j < num_joints; ++j)
	{
		tVector joint_pos = mChar->CalcJointPos(j);
		tVector joint_vel = mChar->CalcJointVel(j);
		joint_pos -= root_pos;
		int offset = (j - 1) * gPosDim;
		mPoliState.segment(offset, gPosDim) = joint_pos.segment(0, gPosDim);
		mPoliState.segment(offset + pose_size, gPosDim) = joint_vel.segment(0, gPosDim);
	}

	const auto& joint_mat = mChar->GetJointMat();
	tVector tar_root_pos = cKinTree::GetRootPos(joint_mat, mTargetPose);
	for (int j = 1; j < num_joints; ++j)
	{
		tVector joint_pos = cKinTree::CalcJointWorldPos(joint_mat, mTargetPose, j);
		tVector joint_vel = cKinTree::CalcJointWorldVel(joint_mat, mTargetPose, mTargetVel, j);
		joint_pos -= tar_root_pos;
		int offset = 2 * pose_size + (j - 1) * gPosDim;
		mPoliState.segment(offset, gPosDim) = joint_pos.segment(0, gPosDim);
		mPoliState.segment(offset + pose_size, gPosDim) = joint_vel.segment(0, gPosDim);
	}
}