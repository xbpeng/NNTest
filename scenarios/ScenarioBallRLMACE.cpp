#include "ScenarioBallRLMACE.h"

const int gTrainerPlaybackMemSize = 50000;

cScenarioBallRLMACE::cScenarioBallRLMACE()
{
}

cScenarioBallRLMACE::~cScenarioBallRLMACE()
{
}

void cScenarioBallRLMACE::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);

	parser.ParseString("critic_net_file", mCriticNetFile);
	parser.ParseString("critic_solver_file", mCriticSolverFile);
}

std::string cScenarioBallRLMACE::GetName() const
{
	return "Ball RL MACE";
}

void cScenarioBallRLMACE::SetupController()
{
	std::shared_ptr<cBallController> ctrl;
	BuildController(ctrl);
	ctrl->SetGround(&mGround);
	ctrl->SetCtrlNoise(mCtrlNoise);

	if (mNetFile != "")
	{
		bool succ = ctrl->LoadNet(mNetFile);
		if (!succ)
		{
			printf("Failed to load network from %s\n", mNetFile.c_str());
		}
	}

	if (mModelFile != "")
	{
		ctrl->LoadModel(mModelFile);
	}

	std::shared_ptr<cBallControllerMACE> mace_ctrl = std::static_pointer_cast<cBallControllerMACE>(ctrl);
	if (mCriticNetFile != "")
	{
		mace_ctrl->LoadCriticNet(mCriticNetFile);
	}


	mBall.SetController(ctrl);
}

void cScenarioBallRLMACE::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallControllerMACE>(new cBallControllerMACE(mBall));
}

void cScenarioBallRLMACE::InitTrainer()
{
	std::shared_ptr<cMACETrainer> trainer = std::shared_ptr<cMACETrainer>(new cMACETrainer());
	
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	
	auto ctrl = GetACECtrl();
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());
	trainer->SetActorFiles(mSolverFile, mNetFile);

	trainer->Init(mTrainerParams);

	if (mModelFile != "")
	{
		trainer->LoadCriticModel(mModelFile);
	}

	mTrainer = trainer;
	SetupTrainerOutputOffsetScale();
}

std::shared_ptr<cBallControllerMACE> cScenarioBallRLMACE::GetMACECtrl() const
{
	return std::static_pointer_cast<cBallControllerMACE>(mBall.GetController());
}

std::shared_ptr<cMACETrainer> cScenarioBallRLMACE::GetMACETrainer() const
{
	return std::static_pointer_cast<cMACETrainer>(mTrainer);
}


void cScenarioBallRLMACE::SetupTrainerOutputOffsetScale()
{
	Eigen::VectorXd critic_offset;
	Eigen::VectorXd critic_scale;
	Eigen::VectorXd actor_offset;
	Eigen::VectorXd actor_scale;
	BuildCriticOutputOffsetScale(critic_offset, critic_scale);
	BuildActorOutputOffsetScale(actor_offset, actor_scale);

	auto trainer = GetMACETrainer();
	trainer->SetCriticOutputOffsetScale(critic_offset, critic_scale);
	trainer->SetActorOutputOffsetScale(actor_offset, actor_scale);
}

void cScenarioBallRLMACE::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto trainer = GetMACETrainer();
	int critic_size = trainer->GetCriticOutputSize();
	out_offset = -0.5 * Eigen::VectorXd::Ones(critic_size);
	out_scale = 2 * Eigen::VectorXd::Ones(critic_size);
}

void cScenarioBallRLMACE::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto trainer = GetMACETrainer();
	int actor_size = trainer->GetActorOutputSize();

	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double dist_offset = -0.5 * (max_dist + min_dist);
	double dist_scale = 2 / (max_dist - min_dist);

	out_offset = dist_offset * Eigen::VectorXd::Ones(actor_size);
	out_scale = dist_scale * Eigen::VectorXd::Ones(actor_size);
}