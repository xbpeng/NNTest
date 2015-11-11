#include "ScenarioBallRLCacla.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLCacla::cScenarioBallRLCacla()
{
}

cScenarioBallRLCacla::~cScenarioBallRLCacla()
{
}

void cScenarioBallRLCacla::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);
	parser.ParseString("actor_solver_file", mActorSolverFile);
	parser.ParseString("actor_net_file", mActorNetFile);
}

std::string cScenarioBallRLCacla::GetName() const
{
	return "Ball RL Cacla";
}

void cScenarioBallRLCacla::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCont(mBall));
}

void cScenarioBallRLCacla::InitTrainer()
{
	std::shared_ptr<cCaclaTrainer> trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());

	cCaclaTrainer::tParams params;
	params.mNetFile = mNetFile;
	params.mSolverFile = mSolverFile;

	params.mPlaybackMemSize = gTrainerPlaybackMemSize;
	params.mPoolSize = 1;
	params.mNumInitSamples = 50;
	trainer->Init(params, mActorSolverFile, mActorNetFile);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	mTrainer = trainer;
}
