#include "ScenarioBallRLCacla.h"
#include "learning/ACLearner.h"

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

void cScenarioBallRLCacla::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);
	parser.ParseString("critic_solver_file", mCriticSolverFile);
	parser.ParseString("critic_net_file", mCriticNetFile);
	parser.ParseString("critic_model_file", mCriticModelFile);
}

void cScenarioBallRLCacla::SaveCriticNet(const std::string& filename) const
{
	std::shared_ptr<cCaclaTrainer> trainer = std::static_pointer_cast<cCaclaTrainer>(mTrainer);
	trainer->OutputCritic(filename);
}

void cScenarioBallRLCacla::SaveActorNet(const std::string& filename) const
{
	std::shared_ptr<cCaclaTrainer> trainer = std::static_pointer_cast<cCaclaTrainer>(mTrainer);
	trainer->OutputActor(filename);
}

std::string cScenarioBallRLCacla::GetName() const
{
	return "Ball RL Cacla";
}

void cScenarioBallRLCacla::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCacla(mBall));
}

void cScenarioBallRLCacla::InitTrainer()
{
	std::shared_ptr<cCaclaTrainer> trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	//std::shared_ptr<cCaclaACTrainer> trainer = std::shared_ptr<cCaclaACTrainer>(new cCaclaACTrainer());
	//std::shared_ptr<cCaclaTrainer> trainer = std::shared_ptr<cCaclaTrainer>(new cQCaclaTrainer());

	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	//mTrainerParams.mNetFile = mNetFile;
	//mTrainerParams.mSolverFile = mSolverFile;

	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	//mTrainerParams.mNumInitSamples = 10;
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
	BuildCriticOutputOffsetScale(trainer, critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(trainer, actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	mTrainer = trainer;
}

void cScenarioBallRLCacla::BuildCriticOutputOffsetScale(const std::shared_ptr<cCaclaTrainer>& trainer,
														Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	auto ctrl = std::static_pointer_cast<cBallControllerCacla>(mBall.GetController());
	ctrl->BuildCriticOutputOffsetScale(out_offset, out_scale);
}

void cScenarioBallRLCacla::BuildActorOutputOffsetScale(const std::shared_ptr<cCaclaTrainer>& trainer,
														Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
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