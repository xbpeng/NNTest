#include "ScenarioBallRLMACEDPG.h"
#include "learning/MACEDPGTrainer.h"
#include "stuff/BallControllerMACEDPG.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLMACEDPG::cScenarioBallRLMACEDPG()
{
}

cScenarioBallRLMACEDPG::~cScenarioBallRLMACEDPG()
{
}

void cScenarioBallRLMACEDPG::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRLDPG::ParseArgs(parser);
}

std::string cScenarioBallRLMACEDPG::GetName() const
{
	return "Ball RL MACE DPG";
}

void cScenarioBallRLMACEDPG::InitTrainer()
{
	std::shared_ptr<cMACEDPGTrainer> trainer = std::shared_ptr<cMACEDPGTrainer>(new cMACEDPGTrainer());
	mTrainer = trainer;

	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mFreezeTargetIters = 200;

	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());

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
	BuildCriticOutputOffsetScale(trainer, critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(trainer, actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	int action_size = ctrl->GetActionSize();
	Eigen::VectorXd action_min = ctrl->gMinDist * Eigen::VectorXd::Ones(action_size);
	Eigen::VectorXd action_max = ctrl->gMaxDist * Eigen::VectorXd::Ones(action_size);
	trainer->SetActionBounds(action_min, action_max);

	double temp = GetExpTemp();
	trainer->SetBoltzTemp(temp);
}

void cScenarioBallRLMACEDPG::SetupController()
{
	cScenarioBallRLDPG::SetupController();
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	
	if (mCriticNetFile != "")
	{
		bool succ = ctrl->LoadCriticNet(mCriticNetFile);
		if (!succ)
		{
			printf("Failed to load network from %s\n", mCriticNetFile.c_str());
		}
	}

	if (mCriticModelFile != "")
	{
		ctrl->LoadCriticModel(mCriticModelFile);
	}
}

void cScenarioBallRLMACEDPG::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerMACEDPG(mBall));
}

void cScenarioBallRLMACEDPG::BuildCriticOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
									Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	ctrl->BuildCriticOutputOffsetScale(out_offset, out_scale);
}

void cScenarioBallRLMACEDPG::BuildActorOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
									Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	ctrl->BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cScenarioBallRLMACEDPG::CopyTrainerNets()
{
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	auto trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainer);
	const auto& actor_net = trainer->GetActor();
	const auto& critic_net = trainer->GetCritic();

	ctrl->CopyActorNet(*actor_net.get());
	ctrl->CopyCriticNet(*critic_net.get());
}

void cScenarioBallRLMACEDPG::Train()
{
	cScenarioBallRLDPG::Train();

	double temp = GetExpTemp();
	auto trainer = std::static_pointer_cast<cMACEDPGTrainer>(mTrainer);
	trainer->SetBoltzTemp(temp);
}