#include "ArmPDNNPixelController.h"
#include "SimArm.h"

cArmPDNNPixelController::cArmPDNNPixelController()
{
}

cArmPDNNPixelController::~cArmPDNNPixelController()
{
}

void cArmPDNNPixelController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmNNPixelController::Init(character);

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mImpPDCtrl.Init(mChar, pd_params, gravity);
	}
}

void cArmPDNNPixelController::Reset()
{
	cArmNNPixelController::Reset();
	mImpPDCtrl.Reset();
}

void cArmPDNNPixelController::Clear()
{
	cArmNNPixelController::Clear();
	mImpPDCtrl.Clear();
}

void cArmPDNNPixelController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double theta_scale = 1;
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = theta_scale * Eigen::VectorXd::Ones(output_size);
}

void cArmPDNNPixelController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int output_size = GetPoliActionSize();
	double max_theta = 2 * M_PI;
	out_min = -max_theta * Eigen::VectorXd::Ones(output_size);
	out_max = max_theta * Eigen::VectorXd::Ones(output_size);
}

void cArmPDNNPixelController::UpdatePoliAction()
{
	cArmNNPixelController::UpdatePoliAction();
}

void cArmPDNNPixelController::ApplyPoliAction(double time_step, const tAction& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.mParams.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double theta = action.mParams[idx];
			Eigen::VectorXd tar_theta(1);
			tar_theta[0] = theta;
			mImpPDCtrl.SetTargetTheta(j, tar_theta);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}