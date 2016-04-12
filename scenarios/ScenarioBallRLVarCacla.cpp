#include "ScenarioBallRLVarCacla.h"

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
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCacla(mBall));
}

void cScenarioBallRLVarCacla::InitTrainer()
{
	auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	//mTrainerParams.mFreezeTargetIters = 100;

	trainer->SetActorFiles(mSolverFile, mNetFile);
	trainer->Init(mTrainerParams);

	if (mModelFile != "")
	{
		trainer->LoadActorModel(mModelFile);
	}

	if (mCriticModelFile != "")
	{
		trainer->LoadCriticModel(mCriticModelFile);
	}
	
	Eigen::VectorXd critic_output_offset;
	Eigen::VectorXd critic_output_scale;
	BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	mTrainer = trainer;
}