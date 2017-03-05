#include "ScenarioBallRLMACEDPG.h"
#include "learning/MACEDPGTrainer.h"
#include "stuff/BallControllerMACEDPG.h"

const int gTrainerPlaybackMemSize = 20000;
const double gTrainerTempScale = 1;

cScenarioBallRLMACEDPG::cScenarioBallRLMACEDPG()
{
}

cScenarioBallRLMACEDPG::~cScenarioBallRLMACEDPG()
{
}

void cScenarioBallRLMACEDPG::ParseArgs(const std::shared_ptr<cArgParser>& parser)
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

	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mFreezeTargetIters = 200;
	mTrainerParams.mPretrainIters = 5000;

	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());

	trainer->SetDPGReg(0.1);
	trainer->SetQDiff(0.1);
	trainer->Init(mTrainerParams);
	
	Eigen::VectorXd critic_output_offset;
	Eigen::VectorXd critic_output_scale;
	BuildCriticOutputOffsetScale(trainer, critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(trainer, actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	const auto& exp_params = ctrl->GetExpParams();
	double temp = exp_params.mTemp;
	trainer->SetTemp(gTrainerTempScale * temp);
}

void cScenarioBallRLMACEDPG::SetupController()
{
	cScenarioBallRLDPG::SetupController();
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	
	if (mTrainerParams.mCriticNetFile != "")
	{
		bool succ = ctrl->LoadCriticNet(mTrainerParams.mCriticNetFile);
		if (!succ)
		{
			printf("Failed to load network from %s\n", mTrainerParams.mCriticNetFile.c_str());
		}
	}

	if (mTrainerParams.mCriticModelFile != "")
	{
		ctrl->LoadCriticModel(mTrainerParams.mCriticModelFile);
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

void cScenarioBallRLMACEDPG::Train()
{
	auto ctrl = std::static_pointer_cast<cBallControllerMACEDPG>(mBall.GetController());
	const auto& exp_params = ctrl->GetExpParams();
	double temp = exp_params.mTemp;
	auto learner = std::static_pointer_cast<cMACEDPGLearner>(mLearner);
	learner->SetTemp(gTrainerTempScale * temp);
	cScenarioBallRLDPG::Train();
}