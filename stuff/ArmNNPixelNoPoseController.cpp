#include "ArmNNPixelNoPoseController.h"
#include "SimArm.h"

cArmNNPixelNoPoseController::cArmNNPixelNoPoseController()
{
}

cArmNNPixelNoPoseController::~cArmNNPixelNoPoseController()
{
}

int cArmNNPixelNoPoseController::GetPoliStateSize() const
{
	int pixel_size = GetPixelDim();
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = pixel_size + pose_dim;
	return state_size;
}

void cArmNNPixelNoPoseController::BuildPoliStateOffsetScale(Eigen::VectorXd& out_mean, Eigen::VectorXd& out_stdev) const
{
	const double pixel_mean = 0.5;
	const double pixel_scale = 0.5;
	int pixel_size = GetPixelDim();
	int state_size = GetPoliStateSize();

	out_mean = Eigen::VectorXd::Zero(state_size);
	out_stdev = Eigen::VectorXd::Ones(state_size);

	int pose_size = (state_size - pixel_size);
	out_stdev.segment(pixel_size, pose_size) = (1 / (2 * M_PI)) * Eigen::VectorXd::Ones(pose_size);
}

void cArmNNPixelNoPoseController::UpdatePoliState()
{
	Eigen::VectorXd vel;
	mChar->BuildVel(vel);

	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int param_size = static_cast<int>(vel.size()) - root_size;
	int pixel_size = GetPixelDim();

	mPoliState.segment(0, pixel_size) = mViewBuffer;
	mPoliState.segment(pixel_size, param_size) = vel.segment(root_size, param_size);
}