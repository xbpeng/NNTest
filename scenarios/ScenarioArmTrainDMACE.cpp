#include "ScenarioArmTrainDMACE.h"
#include "learning/DMACETrainer.h"

const int gTrainerPlaybackMemSize = 25000;

cScenarioArmTrainDMACE::cScenarioArmTrainDMACE()
{
}

cScenarioArmTrainDMACE::~cScenarioArmTrainDMACE()
{
}

std::string cScenarioArmTrainDMACE::GetName() const
{
	return "Arm Train D-MACE";
}


void cScenarioArmTrainDMACE::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, cDMACETrainer::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, cDMACETrainer::eFlagExpActor);
}

void cScenarioArmTrainDMACE::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cDMACETrainer::eFlagFail);
}

void cScenarioArmTrainDMACE::InitTrainer()
{
	auto trainer = std::static_pointer_cast<cDMACETrainer>(mTrainer);

	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 20000;
	mTrainerParams.mInitInputOffsetScale = false;
	//mTrainerParams.mFreezeTargetIters = 200;

	auto ctrl = GetMACECtrl();
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());
	trainer->SetActorFiles(mActorSolverFile, mActorNetFile);
	trainer->SetMode(cCaclaTrainer::eModeCacla);
	trainer->SetTDScale(20);
	trainer->SetGateScale(1);

	trainer->Init(mTrainerParams);

	if (mCriticModelFile != "")
	{
		trainer->LoadCriticModel(mCriticModelFile);
	}

	if (mActorModelFile != "")
	{
		trainer->LoadActorModel(mActorModelFile);
	}

	SetupScale();
	SetupActionBounds();
}

void cScenarioArmTrainDMACE::InitLearner()
{
	cScenarioArmTrain::InitLearner();

	double temp = CalcExpTemp();
	auto trainer = std::static_pointer_cast<cDMACETrainer>(mTrainer);
	auto learner = std::static_pointer_cast<cDMACELearner>(mLearner);
	learner->SetTemp(temp);
	trainer->SetTemp(temp);
}

void cScenarioArmTrainDMACE::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	auto trainer = std::shared_ptr<cDMACETrainer>(new cDMACETrainer());
	out_trainer = trainer;
}

void cScenarioArmTrainDMACE::SetupActorScale()
{
	auto trainer = std::static_pointer_cast<cDMACETrainer>(mTrainer);
	if (!trainer->HasActorInitModel())
	{
		auto ctrl = GetController();
		int input_size = trainer->GetActorInputSize();
		int output_size = trainer->GetActorOutputSize();
		int num_actors = trainer->GetNumActionFrags();

		Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(input_size);
		Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(input_size);
		ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
		trainer->SetActorInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(output_size);
		Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(output_size);
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
		output_offset.segment(0, num_actors) = Eigen::VectorXd::Zero(num_actors);
		output_scale.segment(0, num_actors) = Eigen::VectorXd::Ones(num_actors);
		//output_scale.segment(0, num_actors) = 0.1 * Eigen::VectorXd::Ones(num_actors);
		
		trainer->SetActorOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioArmTrainDMACE::SetupCriticScale()
{
	auto trainer = std::static_pointer_cast<cDMACETrainer>(mTrainer);
	if (!trainer->HasCriticInitModel())
	{
		auto ctrl = GetController();
		int input_size = trainer->GetCriticInputSize();
		int output_size = trainer->GetCriticOutputSize();

		Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(input_size);
		Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(input_size);
		ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
		trainer->SetCriticInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
		Eigen::VectorXd output_scale = 2 * Eigen::VectorXd::Ones(output_size);
		trainer->SetCriticOutputOffsetScale(output_offset, output_scale);
	}
}

double cScenarioArmTrainDMACE::CalcExpTemp() const
{
	return cScenarioArmTrainMACE::CalcExpTemp();
}

void cScenarioArmTrainDMACE::Train()
{
	auto learner = std::static_pointer_cast<cDMACELearner>(mLearner);
	learner->SetTemp(CalcExpTemp());
	cScenarioArmTrainMACE::Train();
}