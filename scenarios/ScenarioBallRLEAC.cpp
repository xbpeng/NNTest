#include "ScenarioBallRLEAC.h"

const int gTrainerPlaybackMemSize = 50000;

cScenarioBallRLEAC::cScenarioBallRLEAC()
{
}

cScenarioBallRLEAC::~cScenarioBallRLEAC()
{
}

void cScenarioBallRLEAC::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRL::ParseArgs(parser);
}

std::string cScenarioBallRLEAC::GetName() const
{
	return "Ball RL EAC";
}

void cScenarioBallRLEAC::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerEAC(mBall));
}

void cScenarioBallRLEAC::InitTrainer()
{
	std::shared_ptr<cEACTrainer> trainer = std::shared_ptr<cEACTrainer>(new cEACTrainer());
	
	mTrainerParams.mNetFile = mNetFile;
	mTrainerParams.mSolverFile = mSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	
	auto ctrl = GetEACCtrl();
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


void cScenarioBallRLEAC::BuildOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer,
							Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = trainer->GetOutputSize();
	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	int num_action = GetNumActionFrags();
	int action_size = GetActionFragSize();

	out_offset.segment(0, num_action) *= -0.5;
	out_scale.segment(0, num_action) *= 2;

	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double dist_offset = -0.5 * (max_dist + min_dist);
	double dist_scale = 2 / (max_dist - min_dist);

	out_offset.segment(num_action, num_action * action_size) *= dist_offset;
	out_scale.segment(num_action, num_action * action_size) *= dist_scale;
}

void cScenarioBallRLEAC::RecordBegFlags(tExpTuple& out_tuple) const
{
	bool off_policy = CheckOffPolicy();
	bool explore = CheckExplore();
	out_tuple.SetFlag(off_policy, cEACTrainer::eFlagOffPolicy);
	out_tuple.SetFlag(explore, cEACTrainer::eFlagExplore);
}

bool cScenarioBallRLEAC::CheckOffPolicy() const
{
	const auto& ctrl = mBall.GetController();
	return ctrl->IsOffPolicy();
}

bool cScenarioBallRLEAC::CheckExplore() const
{
	auto ctrl = GetEACCtrl();
	return ctrl->IsExploring();
}

int cScenarioBallRLEAC::GetNumActionFrags() const
{
	auto ctrl = GetEACCtrl();
	return ctrl->GetNumActionFrags();
}

int cScenarioBallRLEAC::GetActionFragSize() const
{
	auto ctrl = GetEACCtrl();
	return ctrl->GetActionFragSize();
}

std::shared_ptr<cBallControllerEAC> cScenarioBallRLEAC::GetEACCtrl() const
{
	return std::static_pointer_cast<cBallControllerEAC>(mBall.GetController());
}