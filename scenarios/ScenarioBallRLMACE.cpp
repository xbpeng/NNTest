#include "ScenarioBallRLMACE.h"
#include "learning/AsyncMACETrainer.h"

const int gTrainerPlaybackMemSize = 50000;

cScenarioBallRLMACE::cScenarioBallRLMACE()
{
	mInitExpRate = 0.9;
}

cScenarioBallRLMACE::~cScenarioBallRLMACE()
{
}

void cScenarioBallRLMACE::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);
}

std::string cScenarioBallRLMACE::GetName() const
{
	return "Ball RL ACE";
}

void cScenarioBallRLMACE::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerMACE(mBall));
}

void cScenarioBallRLMACE::InitTrainer()
{
	//auto trainer = std::shared_ptr<cMACETrainer>(new cMACETrainer());
	auto trainer = std::shared_ptr<cAsyncMACETrainer>(new cAsyncMACETrainer());

	mTrainerParams.mNetFile = mNetFile;
	mTrainerParams.mSolverFile = mSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mFreezeTargetIters = 500;

	auto ctrl = GetACECtrl();
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());
	trainer->Init(mTrainerParams);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	mTrainer = trainer;
	SetupTrainerOutputOffsetScale();
}

void cScenarioBallRLMACE::RecordBegFlags(tExpTuple& out_tuple) const
{
	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, cMACETrainer::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, cMACETrainer::eFlagExpActor);
}

bool cScenarioBallRLMACE::CheckExpCritic() const
{
	auto ctrl = GetACECtrl();
	return ctrl->IsExpCritic();
}

bool cScenarioBallRLMACE::CheckExpActor() const
{
	auto ctrl = GetACECtrl();
	return ctrl->IsExpActor();
}

int cScenarioBallRLMACE::GetNumActionFrags() const
{
	auto ctrl = GetACECtrl();
	return ctrl->GetNumActionFrags();
}

int cScenarioBallRLMACE::GetActionFragSize() const
{
	auto ctrl = GetACECtrl();
	return ctrl->GetActionFragSize();
}

std::shared_ptr<cBallControllerMACE> cScenarioBallRLMACE::GetACECtrl() const
{
	return std::static_pointer_cast<cBallControllerMACE>(mBall.GetController());
}