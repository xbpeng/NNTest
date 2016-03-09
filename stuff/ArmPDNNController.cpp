#include "ArmPDNNController.h"
#include "SimArm.h"

cArmPDNNController::cArmPDNNController()
{
}

cArmPDNNController::~cArmPDNNController()
{
}

void cArmPDNNController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmNNController::Init(character);

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mImpPDCtrl.Init(mChar, pd_params, gravity);
	}
}

void cArmPDNNController::Reset()
{
	cArmNNController::Reset();
	mImpPDCtrl.Reset();
}

void cArmPDNNController::Clear()
{
	cArmNNController::Clear();
	mImpPDCtrl.Clear();
}

void cArmPDNNController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double theta_scale = 1 / (2 * M_PI);
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = theta_scale * Eigen::VectorXd::Ones(output_size);
}

void cArmPDNNController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int output_size = GetPoliActionSize();
	double max_theta = 2 * M_PI;
	out_min = -max_theta * Eigen::VectorXd::Ones(output_size);
	out_max = max_theta * Eigen::VectorXd::Ones(output_size);
}

void cArmPDNNController::UpdatePoliAction()
{
	cArmNNController::UpdatePoliAction();
}

void cArmPDNNController::ApplyPoliAction(double time_step, const tAction& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.mParams.size()) == GetPoliActionSize());

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double theta = action.mParams[idx];
			mImpPDCtrl.SetTargetTheta(j, theta);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}