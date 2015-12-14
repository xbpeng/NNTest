#include "ScenarioBallRLACE.h"

const int gTrainerPlaybackMemSize = 50000;

cScenarioBallRLACE::cScenarioBallRLACE()
{
}

cScenarioBallRLACE::~cScenarioBallRLACE()
{
}

void cScenarioBallRLACE::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);
}

std::string cScenarioBallRLACE::GetName() const
{
	return "Ball RL ACE";
}

void cScenarioBallRLACE::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerACE(mBall));
}

void cScenarioBallRLACE::InitTrainer()
{
	std::shared_ptr<cACETrainer> trainer = std::shared_ptr<cACETrainer>(new cACETrainer());
	
	mTrainerParams.mNetFile = mNetFile;
	mTrainerParams.mSolverFile = mSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	
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


void cScenarioBallRLACE::BuildOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
							Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = trainer->GetOutputSize();
	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	int num_actions = GetNumActionFrags();
	int action_size = GetActionFragSize();

	out_offset.segment(0, num_actions) *= -0.5;
	out_scale.segment(0, num_actions) *= 2;

	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double dist_scale = 2 / (max_dist - min_dist);

	double dist_steps = (max_dist - min_dist) / (num_actions + 1);
	for (int a = 0; a < num_actions; ++a)
	{
		double dist_offset = -(a + 1) * dist_steps;
		out_offset.segment(num_actions + a * action_size, action_size) *= dist_offset;
		out_scale.segment(num_actions + a * action_size, action_size) *= dist_scale;
	}
}

void cScenarioBallRLACE::RecordBegFlags(tExpTuple& out_tuple) const
{
	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, cACETrainer::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, cACETrainer::eFlagExpActor);
}

bool cScenarioBallRLACE::CheckExpCritic() const
{
	auto ctrl = GetACECtrl();
	return ctrl->IsExpCritic();
}

bool cScenarioBallRLACE::CheckExpActor() const
{
	auto ctrl = GetACECtrl();
	return ctrl->IsExpActor();
}

int cScenarioBallRLACE::GetNumActionFrags() const
{
	auto ctrl = GetACECtrl();
	return ctrl->GetNumActionFrags();
}

int cScenarioBallRLACE::GetActionFragSize() const
{
	auto ctrl = GetACECtrl();
	return ctrl->GetActionFragSize();
}

std::shared_ptr<cBallControllerACE> cScenarioBallRLACE::GetACECtrl() const
{
	return std::static_pointer_cast<cBallControllerACE>(mBall.GetController());
}