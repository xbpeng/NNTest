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

void cArmVelNNPixelController::UpdatePoliAction()
{
	cArmNNPixelController::UpdatePoliAction();
	// hack
	mPoliAction(mPoliAction.size() - 1) = 0;
}

void cArmVelNNPixelController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double vel = action[idx];
			vel /= cArmVelQPController::gVelScale;
			mImpPDCtrl.SetTargetVel(j, vel);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}