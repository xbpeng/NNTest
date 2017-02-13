#include "ScenarioBallRLCacla.h"
#include "learning/ACLearner.h"
#include "learning/AsyncCaclaTrainer.h"
#include "stuff/BallControllerCaclaStochastic.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLCacla::cScenarioBallRLCacla()
{
}

cScenarioBallRLCacla::~cScenarioBallRLCacla()
{
}

void cScenarioBallRLCacla::InitLearner()
{
	mTrainer->RequestLearner(mLearner);
	std::shared_ptr<cACLearner> learner = std::static_pointer_cast<cACLearner>(mLearner);

	auto ctrl = std::static_pointer_cast<cBallControllerCacla>(mBall.GetController());
	cNeuralNet& actor_net = ctrl->GetActor();
	cNeuralNet& critic_net = ctrl->GetCritic();
	learner->SetActorNet(&actor_net);
	learner->SetCriticNet(&critic_net);
	learner->Init();
}

void cScenarioBallRLCacla::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioBallRL::ParseArgs(parser);
	parser->ParseString("critic_solver_file", mTrainerParams.mCriticSolverFile);
	parser->ParseString("critic_net_file", mTrainerParams.mCriticNetFile);
	parser->ParseString("critic_model_file", mTrainerParams.mCriticModelFile);
}

void cScenarioBallRLCacla::SaveCriticNet(const std::string& filename) const
{
	std::shared_ptr<cACLearner> learner = std::static_pointer_cast<cACLearner>(mLearner);
	learner->OutputCritic(filename);
}

void cScenarioBallRLCacla::SaveActorNet(const std::string& filename) const
{
	std::shared_ptr<cACLearner> learner = std::static_pointer_cast<cACLearner>(mLearner);
	learner->OutputActor(filename);
}

std::string cScenarioBallRLCacla::GetName() const
{
	return "Ball RL Cacla";
}

void cScenarioBallRLCacla::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	//out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCacla(mBall));
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCaclaStochastic(mBall));
}

void cScenarioBallRLCacla::InitTrainer()
{
	auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	//auto trainer = std::shared_ptr<cAsyncCaclaTrainer>(new cAsyncCaclaTrainer());

	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	//mTrainerParams.mNumInitSamples = 100;
	//mTrainerParams.mFreezeTargetIters = 100;

	mTrainerParams.mPGEnableImportanceSampling = true;
	mTrainerParams.mPGEnableOnPolicy = false;
	mTrainerParams.mPGMode = cCaclaTrainer::ePGModeCacla;
	mTrainerParams.mPGAdvScale = 10;
	mTrainerParams.mPGIWClip = 20;

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

	Eigen::VectorXd action_covar;
	auto ctrl = std::static_pointer_cast<cBallControllerCacla>(mBall.GetController());
	ctrl->BuildActionExpCovar(action_covar);
	trainer->SetActionCovar(action_covar);

	mTrainer = trainer;
}

void cScenarioBallRLCacla::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto ctrl = std::static_pointer_cast<cBallControllerCacla>(mBall.GetController());
	ctrl->BuildCriticOutputOffsetScale(out_offset, out_scale);
}

void cScenarioBallRLCacla::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto ctrl = std::static_pointer_cast<cBallControllerCacla>(mBall.GetController());
	ctrl->BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cScenarioBallRLCacla::RecordBegFlags(tExpTuple& out_tuple) const
{
	bool off_policy = CheckOffPolicy();
	out_tuple.SetFlag(off_policy, cCaclaTrainer::eFlagOffPolicy);
}

bool cScenarioBallRLCacla::CheckOffPolicy() const
{
	const auto& ctrl = mBall.GetController();
	return ctrl->IsOffPolicy();
}

void cScenarioBallRLCacla::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	const auto& ctrl = mBall.GetController();
	ctrl->BuildActionBounds(out_min, out_max);
}