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

void cArmPDNNController::UpdatePoliAction()
{
	cArmNNController::UpdatePoliAction();
	Eigen::VectorXd torques = mPoliAction;
	TorquesToTheta(torques, mPoliAction);
	// hack
	mPoliAction(mPoliAction.size() - 1) = 0;
}

void cArmPDNNController::TorquesToTheta(const Eigen::VectorXd& torques, Eigen::VectorXd& out_theta) const
{
	int idx = 0;
	int num_joints = mImpPDCtrl.GetNumJoints();
	out_theta.resize(torques.size());

	for (int j = 0; j < num_joints; ++j)
	{
		const cPDController& pd_ctrl = mImpPDCtrl.GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			double t = torques[idx] / gTorqueScale;
			double tar_theta = pd_ctrl.CalcTargetTheta(t);
			out_theta[idx] = tar_theta;
			++idx;
		}
	}
}

void cArmPDNNController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
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