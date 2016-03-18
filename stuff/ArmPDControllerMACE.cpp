#include "ArmPDControllerMACE.h"
#include "SimArm.h"

cArmPDControllerMACE::cArmPDControllerMACE()
{
}

cArmPDControllerMACE::~cArmPDControllerMACE()
{
}

void cArmPDControllerMACE::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	cArmControllerMACE::Init(character);

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mImpPDCtrl.Init(mChar, pd_params, gravity);
	}
}

void cArmPDControllerMACE::Reset()
{
	cArmControllerMACE::Reset();
	mImpPDCtrl.Reset();
}

void cArmPDControllerMACE::Clear()
{
	cArmControllerMACE::Clear();
	mImpPDCtrl.Clear();
}

void cArmPDControllerMACE::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double theta_scale = 1 / (2 * M_PI);
	int num_actions = GetNumActionFrags();
	int action_size = GetActionFragSize();
	
	Eigen::VectorXd action_scale = theta_scale * Eigen::VectorXd::Ones(action_size);
	
	int output_size = GetNetOutputSize();
	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	out_offset.segment(0, num_actions) *= -0.5;
	out_scale.segment(0, num_actions) *= 2;

	for (int a = 0; a < num_actions; ++a)
	{
		Eigen::VectorXd curr_offset;
		BuildActorBias(a, curr_offset);
		out_offset.segment(num_actions + a * action_size, action_size) = curr_offset;
		out_scale.segment(num_actions + a * action_size, action_size) = action_scale;
	}
}

void cArmPDControllerMACE::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int output_size = GetPoliActionSize();
	double max_theta = 2 * M_PI;
	out_min = -max_theta * Eigen::VectorXd::Ones(output_size);
	out_max = max_theta * Eigen::VectorXd::Ones(output_size);
}

void cArmPDControllerMACE::BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const
{
	double min = -M_PI * 0.75;
	double max = M_PI * 0.75;

	out_bias = Eigen::VectorXd::Ones(GetActionFragSize());
	int num_actors = GetNumActionFrags();
	if (num_actors < 2)
	{
		out_bias *= 0;
	}
	else
	{
		double lerp = 1 - a_id / (num_actors - 1.0);
		out_bias *= lerp * (max - min) + min;
		//out_bias[0] = lerp * (max - min) + min;
	}
}

void cArmPDControllerMACE::ApplyPoliAction(double time_step, const tAction& action)
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