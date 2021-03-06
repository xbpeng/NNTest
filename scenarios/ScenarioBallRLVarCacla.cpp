#include "ScenarioBallRLVarCacla.h"
#include "learning/VarCaclaTrainer.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLVarCacla::cScenarioBallRLVarCacla()
{
}

cScenarioBallRLVarCacla::~cScenarioBallRLVarCacla()
{
}

std::string cScenarioBallRLVarCacla::GetName() const
{
	return "Ball RL Var Cacla";
}

void cScenarioBallRLVarCacla::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerVarCacla(mBall));
}

void cScenarioBallRLVarCacla::InitTrainer()
{
	auto trainer = std::shared_ptr<cVarCaclaTrainer>(new cVarCaclaTrainer());
	
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	//mTrainerParams.mFreezeTargetIters = 100;

	trainer->Init(mTrainerParams);

	Eigen::VectorXd critic_output_offset;
	Eigen::VectorXd critic_output_scale;
	BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	Eigen::VectorXd action_min;
	Eigen::VectorXd action_max;
	BuildActionBounds(action_min, action_max);
	trainer->SetActionBounds(action_min, action_max);

	mTrainer = trainer;
}