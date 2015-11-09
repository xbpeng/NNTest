#include "ArmVelQPController.h"
#include "SimArm.h"

const double cArmVelQPController::gVelScale = 0.1;

cArmVelQPController::cArmVelQPController()
{
}

cArmVelQPController::~cArmVelQPController()
{
}

void cArmVelQPController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmQPController::Init(character, gravity);

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

void cArmVelQPController::Reset()
{
	cArmQPController::Reset();
	mImpPDCtrl.Reset();
}

void cArmVelQPController::Clear()
{
	cArmQPController::Clear();
	mImpPDCtrl.Clear();
}

void cArmVelQPController::UpdatePoliAction()
{
	cArmQPController::UpdatePoliAction();
	Eigen::VectorXd torques = mPoliAction;
	TorquesToVel(torques, mPoliAction);
	mPoliAction *= gVelScale;
}

void cArmVelQPController::TorquesToVel(const Eigen::VectorXd& torques, Eigen::VectorXd& out_vel) const
{
	int idx = 0;
	int num_joints = mImpPDCtrl.GetNumJoints();
	out_vel.resize(torques.size());

	for (int j = 0; j < num_joints; ++j)
	{
		const cPDController& pd_ctrl = mImpPDCtrl.GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			double t = torques[idx] / gTorqueScale;
			double tar_vel = pd_ctrl.CalcTargetVel(t);
			out_vel[idx] = tar_vel;
			++idx;
		}
	}
}

void cArmVelQPController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	int num_joints = mChar->GetNumJoints();
	assert(static_cast<int>(action.size()) == num_joints - 1);

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		if (mImpPDCtrl.IsValidPDCtrl(j))
		{
			double vel = action[idx];
			vel /= gVelScale;
			mImpPDCtrl.SetTargetVel(j, vel);
			++idx;
		}
	}
	mImpPDCtrl.Update(time_step);
}