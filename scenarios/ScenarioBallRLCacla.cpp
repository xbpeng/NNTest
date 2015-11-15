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
	parser.ParseString("critic_solver_file", mCriticSolverFile);
	parser.ParseString("critic_net_file", mCriticNetFile);
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
	params.mNetFile = mCriticNetFile;
	params.mSolverFile = mCriticSolverFile;

	params.mPlaybackMemSize = gTrainerPlaybackMemSize;
	params.mPoolSize = 1;
	params.mNumInitSamples = 500;
	trainer->Init(params, mSolverFile, mNetFile);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	mTrainer = trainer;
}

void cScenarioBallRLCacla::BuildOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = mTrainer->GetOutputSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);
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