#include "ArmVelNNPixelController.h"
#include "ArmVelQPController.h"
#include "SimArm.h"

cArmVelNNPixelController::cArmVelNNPixelController()
{
}

cArmVelNNPixelController::~cArmVelNNPixelController()
{
}

void cArmVelNNPixelController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmNNPixelController::Init(character);

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mImpPDCtrl.Init(mChar, pd_params, gravity);

		int num_joints = mChar->GetNumJoints();
		for (int j = 0; j < num_joints; ++j)
		{
			if (mImpPDCtrl.IsValidPDCtrl(j))
			{
				mImpPDCtrl.SetKp(j, 0);
			}
		}
	}
}

void cArmVelNNPixelController::Reset()
{
	cArmNNPixelController::Reset();
	mImpPDCtrl.Reset();
}

void cArmVelNNPixelController::Clear()
{
	cArmNNPixelController::Clear();
	mImpPDCtrl.Clear();
}

void cArmVelNNPixelController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double vel_scale = 0.1;
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = vel_scale * Eigen::VectorXd::Ones(output_size);
}

void cArmVelNNPixelController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int output_size = GetPoliActionSize();
	double max_vel = 15 * M_PI;
	out_min = -max_vel * Eigen::VectorXd::Ones(output_size);
	out_max = max_vel * Eigen::VectorXd::Ones(output_size);
}

void cArmVelNNPixelController::UpdatePoliAction()
{
	cArmNNPixelController::UpdatePoliAction();
}

void cArmVelNNPixelController::ApplyPoliAction(double time_step, const tAction& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.mParams.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double vel = action.mParams[idx];
			mImpPDCtrl.SetTargetVel(j, vel);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}