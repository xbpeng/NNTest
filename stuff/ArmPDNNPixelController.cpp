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

void cArmPDNNPixelController::UpdatePoliAction()
{
	cArmNNPixelController::UpdatePoliAction();
	// hack
	mPoliAction(mPoliAction.size() - 1) = 0;
}

void cArmPDNNPixelController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double theta = action[idx];
			mImpPDCtrl.SetTargetTheta(j, theta);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}