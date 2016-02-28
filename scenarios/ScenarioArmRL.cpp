#include "ScenarioArmRL.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTupleBufferSize = 32;
const int gTrainerPlaybackMemSize = 25000;

const double gCamSize = 4;
const int gRTSize = 128;
const double gTargetRadius = 0.15;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const double gTorqueLim = 300;
const double gCtrlUpdatePeriod = 1 / 120.0;

const double gLinearDamping = 0;
const double gAngularDamping = 0;

const std::string gErrFile = "output/arm_rl_err.txt";
const std::string gActionFile = "output/arm_rl_action.txt";

cScenarioArmRL::cScenarioArmRL()
{
	mEnableTraining = true;
	mPretrain = false;

	mCtrlType = eCtrlNN;
	mCoachType = eCtrlQP;
}

cScenarioArmRL::~cScenarioArmRL()
{
}

void cScenarioArmRL::Init()
{
	cScenarioArm::Init();
	BuildCoach();

	InitTrainer();
	InitTupleBuffer();
}

void cScenarioArmRL::ParseArgs(const cArgParser& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseBool("arm_pretrain", mPretrain);

	parser.ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser.ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);

	ParseCtrlType(parser, "coach_type", mCoachType);
}

void cScenarioArmRL::Reset()
{
	mCoach->Reset();
	cScenarioArm::Reset();
}

void cScenarioArmRL::Clear()
{
	cScenarioArm::Clear();
	mCoach->Clear();
	mNumTuples = 0;
}

void cScenarioArmRL::Update(double time_elapsed)
{
	cScenarioArm::Update(time_elapsed);
}

void cScenarioArmRL::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;
}

bool cScenarioArmRL::EnableTraining() const
{
	return mEnableTraining;
}

const std::shared_ptr<cSimCharacter>& cScenarioArmRL::GetCoach() const
{
	return mCoach;
}

void cScenarioArmRL::SaveNet(const std::string& out_file) const
{
	mTrainer.OutputModel(out_file);
}

std::string cScenarioArmRL::GetName() const
{
	return "Arm RL";
}

bool cScenarioArmRL::BuildCoachController(std::shared_ptr<cCharController>& out_ctrl)
{
	std::shared_ptr<cSimCharacter> coach = mCoach;
	return BuildController(coach, mCoachType, out_ctrl);
}

void cScenarioArmRL::BuildCoach()
{
	CreateCharacter(mCoach);

	cSimCharacter::tParams char_params;
	char_params.mPos = GetDefaultCharPos();
	char_params.mCharFile = mCharacterFile;
	char_params.mStateFile = mCharStateFile;
	char_params.mPlaneCons = GetCharPlaneCons();

	bool succ = mCoach->Init(mWorld, char_params);
	if (succ)
	{
		mCoach->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
		InitCharacterPos(mCoach);

		std::shared_ptr<cCharController> ctrl;
		succ = BuildCoachController(ctrl);
		if (succ)
		{
			mCoach->SetController(ctrl);
		}
	}
}

void cScenarioArmRL::UpdateCoach(double time_step)
{
	mCoach->Update(time_step);
}

void cScenarioArmRL::SetCtrlTargetPos(const tVector& target)
{
	cScenarioArm::SetCtrlTargetPos(target);
	auto coach = GetCoachController();
	if (coach != nullptr)
	{
		coach->SetTargetPos(target);
	}
}

void cScenarioArmRL::ApplyRandPose()
{
	cScenarioArm::ApplyRandPose();

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	mCoach->SetPose(pose);
	mCoach->SetVel(vel);
}

int cScenarioArmRL::GetStateSize() const
{
	return mTrainer.GetStateSize();
}

int cScenarioArmRL::GetActionSize() const
{
	return mTrainer.GetActionSize();
}

void cScenarioArmRL::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetStudentController();
	ctrl->RecordPoliState(out_state);
}

void cScenarioArmRL::RecordAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetCoachController();
	ctrl->RecordPoliAction(out_action);
}

void cScenarioArmRL::RecordTuple()
{
	tExpTuple& tuple = mTupleBuffer[mNumTuples];
	RecordState(tuple.mStateBeg);
	RecordAction(tuple.mAction);
	++mNumTuples;
}

void cScenarioArmRL::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	if (new_update)
	{
		UpdateViewBuffer();
		SetNNViewFeatures();
	}
	
	if (EnableSyncCharacters())
	{
		SyncCharacters();
	}

	cScenarioSimChar::UpdateCharacter(time_step);
	UpdateCoach(time_step);

	if (new_update && mEnableTraining)
	{
		bool exploded = HasExploded();
		if (!exploded)
		{
			RecordTuple();

			if (mNumTuples >= static_cast<int>(mTupleBuffer.size()))
			{
				Train();
			}
		}
	}

	if (new_update)
	{
		PrintInfo();
	}
}

void cScenarioArmRL::InitTupleBuffer()
{
	mTupleBuffer.resize(gTupleBufferSize);
	for (int i = 0; i < gTupleBufferSize; ++i)
	{
		mTupleBuffer[i] = tExpTuple(GetStateSize(), GetActionSize());
	}
}

void cScenarioArmRL::InitTrainer()
{
	mTrainerParams.mNetFile = mNetFile;
	mTrainerParams.mSolverFile = mSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	//params.mNumInitSamples = 100;
	mTrainerParams.mInitInputOffsetScale = false;

	mTrainer.Init(mTrainerParams);

	SetupScale();

	if (mModelFiles.size() > 0)
	{
		for (size_t i = 0; i < mModelFiles.size(); ++i)
		{
			mTrainer.LoadModel(mModelFiles[i]);
		}
	}

	if (mScaleFile != "")
	{
		mTrainer.LoadScale(mScaleFile);
	}
}

void cScenarioArmRL::SetupScale()
{
	int state_size = mTrainer.GetStateSize();
	int action_size = mTrainer.GetActionSize();
	if (state_size > 0)
	{
		auto ctrl = GetStudentController();
		Eigen::VectorXd offset = Eigen::VectorXd::Zero(state_size);
		Eigen::VectorXd scale = Eigen::VectorXd::Ones(state_size);
		ctrl->BuildNNInputOffsetScale(offset, scale);
		mTrainer.SetInputOffsetScale(offset, scale);

		Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(action_size);
		Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(action_size);
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
		mTrainer.SetOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioArmRL::Train()
{
	int iter = GetIter();
	int num_tuples = mTrainer.GetNumTuples();
	printf("\nTraining Iter: %i\n", iter);
	printf("Num Tuples: %i\n", num_tuples);

	mTrainer.AddTuples(mTupleBuffer);

	cNeuralNetTrainer::eStage stage0 = mTrainer.GetStage();
	mTrainer.Train();
	cNeuralNetTrainer::eStage stage1 = mTrainer.GetStage();

	const auto& trainer_net = mTrainer.GetNet();
	std::shared_ptr<cArmNNController> ctrl = GetStudentController();
	if (ctrl != nullptr)
	{
		ctrl->CopyNet(*trainer_net.get());
	}

	mNumTuples = 0;
}

int cScenarioArmRL::GetIter() const
{
	return mTrainer.GetIter();
}

std::shared_ptr<cArmQPController> cScenarioArmRL::GetCoachController() const
{
	const auto& coach = mCoach->GetController();
	if (coach == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmQPController>(coach);
}

std::shared_ptr<cArmNNController> cScenarioArmRL::GetStudentController() const
{
	const auto& student = mChar->GetController();
	if (student == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmNNController>(student);
}

void cScenarioArmRL::SyncCharacters()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	
	cNeuralNetTrainer::eStage trainer_stage = mTrainer.GetStage();
	bool sync_student = mPretrain || trainer_stage == cNeuralNetTrainer::eStageInit;

	/*
	if (sync_student && false) // hack
	{
		mCoach->BuildPose(pose);
		mCoach->BuildVel(vel);
		mChar->SetPose(pose);
		mChar->SetVel(vel);
	}
	else
	*/
	{
		mChar->BuildPose(pose);
		mChar->BuildVel(vel);
		mCoach->SetPose(pose);
		mCoach->SetVel(vel);
	}
}

bool cScenarioArmRL::EnableSyncCharacters() const
{
	return mEnableTraining;
}

void cScenarioArmRL::PrintInfo() const
{
	Eigen::VectorXd coach_action;
	Eigen::VectorXd student_action;
	auto coach = GetCoachController();
	auto student = GetStudentController();
	coach->RecordPoliAction(coach_action);
	student->RecordPoliAction(student_action);

	printf("Coach Action: ");
	for (int i = 0; i < coach_action.size(); ++i)
	{
		printf("%.3f\t", coach_action[i]);
	}
	printf("\n");

	printf("Student Action: ");
	for (int i = 0; i < student_action.size(); ++i)
	{
		printf("%.3f\t", student_action[i]);
	}
	printf("\n");

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	printf("Student Pose: ");
	for (int i = 3; i < pose.size(); ++i)
	{
		printf("%.3f\t", pose[i]);
	}
	printf("\n");

	printf("Student Vel: ");
	for (int i = 3; i < vel.size(); ++i)
	{
		printf("%.3f\t", vel[i]);
	}
	printf("\n");

	printf("\n");
}