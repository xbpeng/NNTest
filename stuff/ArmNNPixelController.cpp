#include "ArmNNPixelController.h"
#include "SimArm.h"

const int gViewBufferRes = 128;

cArmNNPixelController::cArmNNPixelController()
{
	mViewBuffer = Eigen::VectorXd::Zero(GetPixelDim());
}

cArmNNPixelController::~cArmNNPixelController()
{
}

int cArmNNPixelController::GetPoliStateSize() const
{
	int pixel_size = GetPixelDim();
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = pixel_size + 2 * pose_dim;
	return state_size;
}

void cArmNNPixelController::BuildNNInputOffsetScale(Eigen::VectorXd& out_mean, Eigen::VectorXd& out_stdev) const
{
	const double pixel_mean = 0.5;
	const double pixel_scale = 0.5;
	int pixel_size = GetPixelDim();
	int state_size = GetPoliStateSize();

	out_mean = Eigen::VectorXd::Zero(state_size);
	out_stdev = Eigen::VectorXd::Ones(state_size);

	int pose_size = (state_size - pixel_size) / 2;

	//out_mean.segment(0, pixel_size) = pixel_mean * Eigen::VectorXd::Ones(pixel_size);
	//out_stdev.segment(0, pixel_size) = pixel_scale * Eigen::VectorXd::Ones(pixel_size);
	out_stdev.segment(pixel_size, pose_size) = (1 / M_PI) * Eigen::VectorXd::Ones(pose_size);
	out_stdev.segment(pixel_size + pose_size, pose_size) = (1 / (2 * M_PI)) * Eigen::VectorXd::Ones(pose_size);
}

int cArmNNPixelController::GetViewBufferRes()
{
	return gViewBufferRes;
}

void cArmNNPixelController::SetViewBuffer(const Eigen::VectorXd& view_buff)
{
	assert(view_buff.size() == mViewBuffer.size());
	mViewBuffer = view_buff;
}

void cArmNNPixelController::UpdatePoliState()
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int param_size = static_cast<int>(pose.size()) - root_size;
	int pixel_size = GetPixelDim();

	mPoliState.segment(0, pixel_size) = mViewBuffer;
	mPoliState.segment(pixel_size, param_size) = pose.segment(root_size, param_size);
	mPoliState.segment(pixel_size + param_size, param_size) = vel.segment(root_size, param_size);
}

int cArmNNPixelController::GetPixelDim() const
{
	int res = GetViewBufferRes();
	return res * res;
}