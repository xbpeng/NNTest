#include "ArmPDQPController.h"
#include "SimArm.h"

cArmPDQPController::cArmPDQPController()
{
}

cArmPDQPController::~cArmPDQPController()
{
}

void cArmPDQPController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmQPController::Init(character, gravity);

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mImpPDCtrl.Init(mChar, pd_params, gravity);
	}
}

void cArmPDQPController::Reset()
{
	cArmQPController::Reset();
	mImpPDCtrl.Reset();
}

void cArmPDQPController::Clear()
{
	cArmQPController::Clear();
	mImpPDCtrl.Clear();
}

void cArmPDQPController::UpdatePoliAction()
{
	cArmQPController::UpdatePoliAction();
	Eigen::VectorXd torques = mPoliAction.mParams;
	TorquesToTheta(torques, mPoliAction.mParams);
}

void cArmPDQPController::TorquesToTheta(const Eigen::VectorXd& torques, Eigen::VectorXd& out_theta) const
{
	int idx = 0;
	int num_joints = mImpPDCtrl.GetNumJoints();
	out_theta.resize(torques.size());

	for (int j = 0; j < num_joints; ++j)
	{
		const cPDController& pd_ctrl = mImpPDCtrl.GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			double t = torques[idx];
			double tar_theta = pd_ctrl.CalcTargetTheta(t);
			out_theta[idx] = tar_theta;
			++idx;
		}
	}
}

void cArmPDQPController::ApplyPoliAction(double time_step, const tAction& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.mParams.size()) == num_joints - 1);

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