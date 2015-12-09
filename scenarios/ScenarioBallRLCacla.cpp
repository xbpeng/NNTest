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
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerCont(mBall));
	//out_ctrl = std::shared_ptr<cBallController>(new cBallControllerContAC(mBall));
}

void cScenarioBallRLCacla::InitTrainer()
{
	std::shared_ptr<cCaclaTrainer> trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	//std::shared_ptr<cCaclaACTrainer> trainer = std::shared_ptr<cCaclaACTrainer>(new cCaclaACTrainer());
	//std::shared_ptr<cCaclaTrainer> trainer = std::shared_ptr<cCaclaTrainer>(new cQCaclaTrainer());

	cCaclaTrainer::tParams params;
	params.mNetFile = mCriticNetFile;
	params.mSolverFile = mCriticSolverFile;
	//params.mNetFile = mNetFile;
	//params.mSolverFile = mSolverFile;

	params.mPlaybackMemSize = gTrainerPlaybackMemSize;
	params.mPoolSize = 1;
	params.mNumInitSamples = 10000;
	//params.mNumInitSamples = 100;
	//params.mFreezeTargetIters = 500;
	trainer->SetActorFiles(mSolverFile, mNetFile);
	trainer->Init(params);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	
	Eigen::VectorXd critic_output_offset;
	Eigen::VectorXd critic_output_scale;
	BuildCriticOutputOffsetScale(trainer, critic_output_offset, critic_output_scale);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

	Eigen::VectorXd actor_output_offset;
	Eigen::VectorXd actor_output_scale;
	BuildActorOutputOffsetScale(trainer, actor_output_offset, actor_output_scale);
	trainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
	
	/*
	Eigen::VectorXd output_offset;
	Eigen::VectorXd output_scale;
	BuildACOutputOffseScale(trainer, output_offset, output_scale);
	trainer->SetOutputOffsetScale(output_offset, output_scale);
	*/

	mTrainer = trainer;
}

void cScenarioBallRLCacla::BuildCriticOutputOffsetScale(const std::shared_ptr<cCaclaTrainer>& trainer,
														Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = trainer->GetCriticOutputSize();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

void cScenarioBallRLCacla::BuildActorOutputOffsetScale(const std::shared_ptr<cCaclaTrainer>& trainer,
														Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = trainer->GetActorOutputSize();
	double min_dist = cBallControllerCont::gMinDist;
	double max_dist = cBallControllerCont::gMaxDist;
	double offset = -0.5 * (max_dist + min_dist);
	double scale = 2 / (max_dist - min_dist);
	out_offset = offset * Eigen::VectorXd::Ones(output_size);
	out_scale = scale * Eigen::VectorXd::Ones(output_size);
}

void cScenarioBallRLCacla::BuildACOutputOffseScale(const std::shared_ptr<cCaclaACTrainer>& trainer,
													Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	// hack so much hack
	int output_size = trainer->GetOutputSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	out_offset[0] = -0.5;
	out_scale[0] = 2;

	double min_dist = cBallControllerCont::gMinDist;
	double max_dist = cBallControllerCont::gMaxDist;
	double dist_offset = -0.5 * (max_dist + min_dist);
	double dist_scale = 2 / (max_dist - min_dist);
	out_offset.segment(1, output_size - 1) = dist_offset * Eigen::VectorXd::Ones(output_size - 1);
	out_scale.segment(1, output_size - 1) = dist_scale * Eigen::VectorXd::Ones(output_size - 1);
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