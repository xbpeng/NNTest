#include "ScenarioBallRLMACE.h"

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
	std::shared_ptr<cMACETrainer> trainer = std::shared_ptr<cMACETrainer>(new cMACETrainer());
	
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

	// hack hack hack
	/*
	auto test_net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	test_net->LoadNet("data/ball_rl/nets/linear_test_deploy.prototxt");
	test_net->LoadSolver("data/ball_rl/nets/linear_test_solver.prototxt");
	
	const auto& params = test_net->GetParams();
	cNeuralNet::tNNData blob0_data[] = { 1 };
	cNeuralNet::tNNData blob1_data[] = { 0 };
	cNeuralNet::tNNData blob2_data[] = { 0.2 };
	cNeuralNet::tNNData blob3_data[] = { 0 };
	params[0]->set_cpu_data(blob0_data);
	params[1]->set_cpu_data(blob1_data);
	params[2]->set_cpu_data(blob2_data);
	params[3]->set_cpu_data(blob3_data);

	test_net->PrintParams();
	Eigen::VectorXd test_x = 0.1 * Eigen::VectorXd::Ones(1);
	Eigen::VectorXd test_y = 0.01 * Eigen::VectorXd::Ones(1);
	Eigen::VectorXd out_x;
	Eigen::VectorXd out_y;
	test_net->Eval(test_x, out_y);
	test_net->Backward(test_y, out_x);

	int xx = 0;
	++xx;
	*/
}


void cScenarioBallRLMACE::BuildOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
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