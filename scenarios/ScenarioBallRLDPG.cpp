#include "ScenarioBallRLDPG.h"
#include "learning/DPGTrainer.h"
#include "stuff/BallControllerDPG.h"

const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRLDPG::cScenarioBallRLDPG()
{
}

cScenarioBallRLDPG::~cScenarioBallRLDPG()
{
}

void cScenarioBallRLDPG::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRLCacla::ParseArgs(parser);
}

std::string cScenarioBallRLDPG::GetName() const
{
	return "Ball RL DPG";
}

void cScenarioBallRLDPG::InitTrainer()
{
	std::shared_ptr<cDPGTrainer> trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
	
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mFreezeTargetIters = 200;

	trainer->SetPretrainIters(5000);
	trainer->SetDPGReg(0.1);
	trainer->SetQDiff(0.1);
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

	/*
	// hack hack hack
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
	Eigen::VectorXd test_y = 0.0 * Eigen::VectorXd::Ones(1);
	Eigen::VectorXd out_x;
	Eigen::VectorXd out_y;
	test_net->Eval(test_x, out_y);
	test_net->Backward(test_y, out_x);

	int xx = 0;
	++xx;
	*/
}

void cScenarioBallRLDPG::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallControllerDPG(mBall));
}

void cScenarioBallRLDPG::NewCycleUpdate()
{
	cScenarioBallRL::NewCycleUpdate();

	if (!mEnableTraining)
	{
		// hack huge hack
		const auto& ctrl = mBall.GetController();

		Eigen::VectorXd state;
		Eigen::VectorXd action;
		ctrl->RecordState(state);
		ctrl->RecordAction(action);

		Eigen::VectorXd x;
		Eigen::VectorXd y;

		x.resize(state.size() + action.size());
		x.segment(0, state.size()) = state;
		x.segment(state.size(), action.size()) = action;

		std::shared_ptr<cDPGTrainer> trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
		const auto& critic = trainer->GetCritic();
		critic->Eval(x, y);

		y = 1 * Eigen::VectorXd::Ones(y.size());
		critic->Backward(y, x);

		Eigen::VectorXd dpg = x.segment(state.size(), action.size());
		printf("DPG: ");
		for (int i = 0; i < dpg.size(); ++i)
		{
			printf("%.3f\t", dpg[i]);
		}
		printf("\n");
	}
}