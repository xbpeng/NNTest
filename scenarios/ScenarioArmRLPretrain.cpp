#include "ScenarioArmRLPretrain.h"

cScenarioArmRLPretrain::cScenarioArmRLPretrain()
{
}

cScenarioArmRLPretrain::~cScenarioArmRLPretrain()
{
}

std::string cScenarioArmRLPretrain::GetName() const
{
	return "Arm RL Pretrain";
}

bool cScenarioArmRLPretrain::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	out_ctrl = nullptr;
	return true;
}

void cScenarioArmRLPretrain::SetupScale()
{
	int state_size = mTrainer.GetStateSize();
	if (state_size > 0)
	{
		Eigen::VectorXd mean = Eigen::VectorXd::Zero(state_size);
		Eigen::VectorXd stdev = Eigen::VectorXd::Ones(state_size);
		mTrainer.SetScale(mean, stdev);
	}
}

bool cScenarioArmRLPretrain::NeedCtrlUpdate() const
{
	const auto ctrl = GetCoachController();
	return ctrl->NeedUpdate();
}

bool cScenarioArmRLPretrain::NeedViewBuffer() const
{
	return true;
}

void cScenarioArmRLPretrain::SetNNViewFeatures()
{
}

void cScenarioArmRLPretrain::RecordState(Eigen::VectorXd& out_state) const
{
	out_state = mViewBuffer;
}

void cScenarioArmRLPretrain::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = mTargetPos.segment(0, 2);
}

void cScenarioArmRLPretrain::PrintInfo() const
{
	const cNeuralNet& net = mTrainer.GetNet();

	Eigen::VectorXd state;
	RecordState(state);

	Eigen::VectorXd net_pos;
	net.Eval(state, net_pos);

	printf("Target Pos: %.5f, %.5f\n", mTargetPos[0], mTargetPos[1]);
	printf("Predicted Pos: %.5f, %.5f\n", net_pos[0], net_pos[1]);
}

void cScenarioArmRLPretrain::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 1;
	out_max = 4;
}

void cScenarioArmRLPretrain::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 3;
	out_max = 4;
}