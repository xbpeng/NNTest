#include "ArmVelNNController.h"
#include "SimArm.h"

cArmVelNNController::cArmVelNNController()
{
}

cArmVelNNController::~cArmVelNNController()
{
}

void cArmVelNNController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmNNController::Init(character);

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

void cArmVelNNController::Reset()
{
	cArmNNController::Reset();
	mImpPDCtrl.Reset();
}

void cArmVelNNController::Clear()
{
	cArmNNController::Clear();
	mImpPDCtrl.Clear();
}

void cArmVelNNController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double vel_scale = 0.1;
	int output_size = GetPoliActionSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = vel_scale * Eigen::VectorXd::Ones(output_size);
}

void cArmVelNNController::UpdatePoliAction()
{
	cArmNNController::UpdatePoliAction();
}

void cArmVelNNController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double vel = action[idx];
			mImpPDCtrl.SetTargetVel(j, vel);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}