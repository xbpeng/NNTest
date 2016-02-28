#include "ScenarioArmTrain.h"
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

cScenarioArmTrain::cScenarioArmTrain()
{
	mEnableTraining = true;
	mPretrain = false;
	mValidSample = false;

	mCtrlType = eCtrlNN;
}

cScenarioArmTrain::~cScenarioArmTrain()
{
}

void cScenarioArmTrain::Init()
{
	cScenarioArm::Init();

	BuildTrainer(mTrainer);
	InitTrainer();
	InitTupleBuffer();
	UpdatePolicy();
	mValidSample = false;
}

void cScenarioArmTrain::ParseArgs(const cArgParser& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser.ParseBool("arm_pretrain", mPretrain);
	parser.ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser.ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);

	parser.ParseString("critic_solver_file", mCriticSolverFile);
	parser.ParseString("critic_net_file", mCriticNetFile);
	parser.ParseString("critic_model_file", mCriticModelFile);
	parser.ParseString("solver_file", mActorSolverFile);
	
	mActorNetFile = mNetFile;

	if (mModelFiles.size() > 0)
	{
		mActorModelFile = mModelFiles[0];
	}
}

void cScenarioArmTrain::Reset()
{
	cScenarioArm::Reset();
	mValidSample = false;
}

void cScenarioArmTrain::Clear()
{
	cScenarioArm::Clear();
	mNumTuples = 0;
	mValidSample = false;
	mTrainer.reset();
}

void cScenarioArmTrain::Update(double time_elapsed)
{
	cScenarioArm::Update(time_elapsed);
}

void cScenarioArmTrain::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;
}

bool cScenarioArmTrain::EnableTraining() const
{
	return mEnableTraining;
}

void cScenarioArmTrain::SaveNet(const std::string& out_file) const
{
	mTrainer->OutputModel(out_file);
}

std::string cScenarioArmTrain::GetName() const
{
	return "Arm Train";
}

void cScenarioArmTrain::SetCtrlTargetPos(const tVector& target)
{
	cScenarioArm::SetCtrlTargetPos(target);
}

void cScenarioArmTrain::ApplyRandPose()
{
	cScenarioArm::ApplyRandPose();

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);
}

void cScenarioArmTrain::RandReset()
{
	cScenarioArm::RandReset();
	mValidSample = false;
}

int cScenarioArmTrain::GetStateSize() const
{
	return mTrainer->GetStateSize();
}

int cScenarioArmTrain::GetActionSize() const
{
	return mTrainer->GetActionSize();
}

void cScenarioArmTrain::RecordTuple(const tExpTuple& tuple)
{
	int buffer_size = static_cast<int>(mTupleBuffer.size());
	if (mNumTuples == buffer_size)
	{
		mNumTuples = 0;
	}
	mTupleBuffer[mNumTuples] = tuple;
	mNumTuples = (mNumTuples + 1) % (buffer_size + 1);
}

void cScenarioArmTrain::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetController();
	ctrl->RecordPoliState(out_state);
}

void cScenarioArmTrain::RecordAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetController();
	ctrl->RecordPoliAction(out_action);
}

double cScenarioArmTrain::CalcReward() const
{
	double reward = 0;
	return reward;
}

bool cScenarioArmTrain::CheckFail() const
{
	bool exploded = HasExploded();
	return exploded;
}


void cScenarioArmTrain::ClearFlags(tExpTuple& out_tuple) const
{
	out_tuple.ClearFlags();
}

void cScenarioArmTrain::RecordFlagsBeg(tExpTuple& out_tuple) const
{
}

void cScenarioArmTrain::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cDPGTrainer::eFlagFail);
}

void cScenarioArmTrain::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	if (new_update)
	{
		RecordState(mCurrTuple.mStateEnd);
		RecordFlagsEnd(mCurrTuple);
		mCurrTuple.mReward = CalcReward();

		if (mValidSample)
		{
			RecordTuple(mCurrTuple);
		}

		if (mNumTuples >= static_cast<int>(mTupleBuffer.size()))
		{
			Train();
		}

		UpdateViewBuffer();
		SetNNViewFeatures();
	}
	
	cScenarioSimChar::UpdateCharacter(time_step);

	if (new_update)
	{
		mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
		RecordAction(mCurrTuple.mAction);
		ClearFlags(mCurrTuple);
		RecordFlagsBeg(mCurrTuple);
		mValidSample = true;
	}

	if (new_update)
	{
		PrintInfo();
	}
}

void cScenarioArmTrain::InitTupleBuffer()
{
	mTupleBuffer.resize(gTupleBufferSize);
	for (int i = 0; i < gTupleBufferSize; ++i)
	{
		mTupleBuffer[i] = tExpTuple(GetStateSize(), GetActionSize());
	}
}

void cScenarioArmTrain::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	auto trainer = std::shared_ptr<cDPGTrainer>(new cDPGTrainer());
	trainer->SetActorFiles(mActorSolverFile, mActorNetFile);
	trainer->SetDPGReg(0.001);
	trainer->SetPretrainIters(10000);
	out_trainer = trainer;
}

void cScenarioArmTrain::InitTrainer()
{
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mInitInputOffsetScale = false;

	mTrainer->Init(mTrainerParams);
	SetupScale();
	SetupActionBounds();

	if (mModelFiles.size() > 0)
	{
		for (size_t i = 0; i < mModelFiles.size(); ++i)
		{
			mTrainer->LoadModel(mModelFiles[i]);
		}
	}

	if (mScaleFile != "")
	{
		mTrainer->LoadScale(mScaleFile);
	}
}

void cScenarioArmTrain::SetupScale()
{
	SetupActorScale();
	SetupCriticScale();
}

void cScenarioArmTrain::SetupActorScale()
{
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	auto ctrl = GetController();
	int state_size = trainer->GetInputSize();
	int action_size = trainer->GetOutputSize();
	
	Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(state_size);
	Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(state_size);
	ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
	trainer->SetActorInputOffsetScale(input_offset, input_scale);

	Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(action_size);
	ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
	trainer->SetActorOutputOffsetScale(output_offset, output_scale);
}

void cScenarioArmTrain::SetupCriticScale()
{
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	auto ctrl = GetController();
	int state_size = trainer->GetStateSize();
	int action_size = trainer->GetActionSize();
	int critic_input_size = trainer->GetCriticInputSize();
	int critic_output_size = trainer->GetCriticOutputSize();

	Eigen::VectorXd state_offset = Eigen::VectorXd::Zero(state_size);
	Eigen::VectorXd state_scale = Eigen::VectorXd::Ones(state_size);
	ctrl->BuildNNInputOffsetScale(state_offset, state_scale);

	Eigen::VectorXd action_offset = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd action_scale = Eigen::VectorXd::Ones(action_size);
	ctrl->BuildNNOutputOffsetScale(action_offset, action_scale);

	Eigen::VectorXd critic_input_offset = Eigen::VectorXd::Zero(state_size + action_size);
	Eigen::VectorXd critic_input_scale = Eigen::VectorXd::Zero(state_size + action_size);
	critic_input_offset.segment(0, state_size) = state_offset;
	critic_input_offset.segment(state_size, action_size) = action_offset;
	critic_input_scale.segment(0, state_size) = state_scale;
	critic_input_scale.segment(state_size, action_size) = action_scale;

	assert(critic_input_offset.size() == critic_input_size);
	assert(critic_input_scale.size() == critic_input_size);
	trainer->SetCriticInputOffsetScale(critic_input_offset, critic_input_scale);

	Eigen::VectorXd critic_output_offset = -0.5 * Eigen::VectorXd::Ones(critic_output_size);
	Eigen::VectorXd critic_output_scale = 2 * Eigen::VectorXd::Ones(critic_output_size);
	trainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
}

void cScenarioArmTrain::SetupActionBounds()
{
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);

	Eigen::VectorXd action_min;
	Eigen::VectorXd action_max;
	BuildDPGBounds(action_min, action_max);
	trainer->SetActionBounds(action_min, action_max);
}

void cScenarioArmTrain::UpdatePolicy()
{
	const auto& trainer_net = mTrainer->GetNet();
	std::shared_ptr<cArmNNController> ctrl = GetController();
	if (ctrl != nullptr)
	{
		ctrl->CopyNet(*trainer_net.get());
	}
}

void cScenarioArmTrain::BuildDPGBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	auto ctrl = GetController();
	ctrl->BuildActionBounds(out_min, out_max);
}

void cScenarioArmTrain::Train()
{
	int iter = GetIter();
	int num_tuples = mTrainer->GetNumTuples();
	printf("\nTraining Iter: %i\n", iter);
	printf("Num Tuples: %i\n", num_tuples);

	mTrainer->AddTuples(mTupleBuffer);

	mTrainer->Train();
	UpdatePolicy();
	mNumTuples = 0;
}

int cScenarioArmTrain::GetIter() const
{
	return mTrainer->GetIter();
}

std::shared_ptr<cArmNNController> cScenarioArmTrain::GetController() const
{
	const auto& student = mChar->GetController();
	if (student == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmNNController>(student);
}

void cScenarioArmTrain::PrintInfo() const
{
	Eigen::VectorXd student_action;
	auto student = GetController();
	student->RecordPoliAction(student_action);

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