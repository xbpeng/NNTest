#include "ScenarioBallRLDPG.h"
#include "learning/DPGTrainer.h"
#include "stuff/BallControllerDPG.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLDPG::cScenarioBallRLDPG()
{
}

cScenarioBallRLDPG::~cScenarioBallRLDPG()
{
}

void cScenarioBallRLDPG::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRLCacla::ParseArgs(parser);
}

std::string cScenarioBallRLDPG::GetName() const
{
	return "Ball RL DPG";
}

void cScenarioBallRLDPG::InitTrainer()
{
	std::shared_ptr<cDPGTrainer> trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
	
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 1000;
	
	trainer->SetActorFiles(mSolverFile, mNetFile);
	trainer->Init(mTrainerParams);

	if (mModelFile != "")
	{
		trainer->LoadActorModel(mModelFile);
	}
	
	Eigen::VectorXd critic_output_offset;
	Eigen::VectorXd critic_output_scale;
	BuildCriticOutputOffsetScale(trainer, critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(trainer, actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	mTrainer = trainer;
}

void cScenarioBallRLDPG::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerDPG(mBall));
}