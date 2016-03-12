#include "ScenarioArmTrainMACE.h"
#include "learning/MACETrainer.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTrainerPlaybackMemSize = 500000; // 25000;

cScenarioArmTrainMACE::cScenarioArmTrainMACE()
{
}

cScenarioArmTrainMACE::~cScenarioArmTrainMACE()
{
}

void cScenarioArmTrainMACE::Init()
{
	assert(mCtrlType == eCtrlMACE || mCtrlType == eCtrlPDMACE);
	cScenarioArmTrain::Init();
}

std::string cScenarioArmTrainMACE::GetName() const
{
	return "Arm Train MACE";
}

void cScenarioArmTrainMACE::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, cMACETrainer::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, cMACETrainer::eFlagExpActor);
}

void cScenarioArmTrainMACE::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cMACETrainer::eFlagFail);
}

void cScenarioArmTrainMACE::InitTrainer()
{
	auto trainer = std::static_pointer_cast<cMACETrainer>(mTrainer);

	mTrainerParams.mNetFile = mActorNetFile;
	mTrainerParams.mSolverFile = mActorSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 20000;
	mTrainerParams.mInitInputOffsetScale = false;
	mTrainerParams.mFreezeTargetIters = 500;

	auto ctrl = GetMACECtrl();
	trainer->SetNumActionFrags(ctrl->GetNumActionFrags());
	trainer->SetActionFragSize(ctrl->GetActionFragSize());
	trainer->Init(mTrainerParams);

	if (mActorModelFile != "")
	{
		trainer->LoadModel(mActorModelFile);
	}

	SetupScale();
}

void cScenarioArmTrainMACE::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	auto trainer = std::shared_ptr<cMACETrainer>(new cMACETrainer());
	out_trainer = trainer;
}

void cScenarioArmTrainMACE::SetupActorScale()
{
	auto trainer = std::static_pointer_cast<cMACETrainer>(mTrainer);
	if (!trainer->HasInitModel())
	{
		auto ctrl = GetController();
		int state_size = trainer->GetInputSize();
		int action_size = trainer->GetOutputSize();

		Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(state_size);
		Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(state_size);
		ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
		trainer->SetInputOffsetScale(input_offset, input_scale);

		Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(action_size);
		Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(action_size);
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
		trainer->SetOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioArmTrainMACE::SetupCriticScale()
{
	// no critic
}

void cScenarioArmTrainMACE::PrintInfo() const
{
	cScenarioArmTrain::PrintInfo();
}


bool cScenarioArmTrainMACE::CheckExpCritic() const
{
	auto ctrl = GetMACECtrl();
	return ctrl->IsExpCritic();
}

bool cScenarioArmTrainMACE::CheckExpActor() const
{
	auto ctrl = GetMACECtrl();
	return ctrl->IsExpActor();
}

int cScenarioArmTrainMACE::GetNumActionFrags() const
{
	auto ctrl = GetMACECtrl();
	return ctrl->GetNumActionFrags();
}

int cScenarioArmTrainMACE::GetActionFragSize() const
{
	auto ctrl = GetMACECtrl();
	return ctrl->GetActionFragSize();
}

std::shared_ptr<cArmControllerMACE> cScenarioArmTrainMACE::GetMACECtrl() const
{
	return std::static_pointer_cast<cArmControllerMACE>(mChar->GetController());
}