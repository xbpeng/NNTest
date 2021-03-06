#include "ScenarioArmTrain.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTupleBufferSize = 32;
const int gTrainerPlaybackMemSize = 25000; // 500000;
const double gTupleRecordRate = 1; // 0.1;

cScenarioArmTrain::cScenarioArmTrain()
{
	mEnableTraining = true;
	mPretrain = false;
	mValidSample = false;

	mExpRate = 0.2;
	mInitExpRate = 1;
	mExpTemp = 0.025;
	mInitExpTemp = 20;

	mNumAnnealIters = 1000;
	//mEnableRandPose = false;

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
	InitLearner();

	InitTupleBuffer();
	UpdatePolicy();
	mValidSample = false;

	auto controller = GetController();
	controller->EnableExp(true);
}

void cScenarioArmTrain::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser->ParseBool("arm_pretrain", mPretrain);
	parser->ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser->ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);
	parser->ParseDouble("exp_rate", mExpRate);
	parser->ParseDouble("init_exp_rate", mInitExpRate);
	parser->ParseDouble("exp_temp", mExpTemp);
	parser->ParseDouble("init_exp_temp", mInitExpTemp);
	parser->ParseInt("num_exp_anneal_iters", mNumAnnealIters);

	parser->ParseString("critic_solver_file", mTrainerParams.mCriticSolverFile);
	parser->ParseString("critic_net_file", mTrainerParams.mCriticNetFile);
	parser->ParseString("critic_model_file", mTrainerParams.mCriticModelFile);
	parser->ParseString("solver_file", mTrainerParams.mSolverFile);

	mTrainerParams.mNetFile = mNetFile;

	if (mModelFiles.size() > 0)
	{
		mTrainerParams.mModelFile = mModelFiles[0];
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
	mLearner.reset();
}

void cScenarioArmTrain::Update(double time_elapsed)
{
	cScenarioArm::Update(time_elapsed);
}

void cScenarioArmTrain::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;

	auto ctrl = GetController();
	ctrl->EnableExp(mEnableTraining);
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
	mValidSample = false;
}

void cScenarioArmTrain::SetRandTarget()
{
	cScenarioArm::SetRandTarget();
	mValidSample = false;
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
	if (cMathUtil::FlipCoin(gTupleRecordRate))
	{
		int buffer_size = static_cast<int>(mTupleBuffer.size());
		int idx = mNumTuples % buffer_size;
		mTupleBuffer[idx] = tuple;
		++mNumTuples;
	}
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
	double tar_w = 0.8;
	double pose_w = 0.1;
	double vel_w = 0.1;

	int end_id = GetEndEffectorID();
	const tVector& tar_pos = GetTargetPos();
	tVector end_pos = mChar->CalcJointPos(end_id);

	tVector delta = mTargetPos - end_pos;
	double tar_reward = exp(-2 * delta.squaredNorm());

	Eigen::VectorXd pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

	// hack
	if (pose.size() > 3 && tar_w > 0)
	{
		// let base joint be whatever pose it wants
		pose[3] = 0;
	}

	double pose_reward = exp(-2 * pose.squaredNorm() / pose.size());
	double vel_reward = exp(-0.1 * vel.squaredNorm() / pose.size());

	double reward = tar_w * tar_reward
		+ pose_w * pose_reward
		+ vel_w * vel_reward;

	if (CheckFail())
	{
		reward = 0;
	}

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
	bool off_policy = CheckOffPolicy();
	out_tuple.SetFlag(off_policy, tExpTuple::eFlagOffPolicy);
}

void cScenarioArmTrain::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, tExpTuple::eFlagFail);
}

void cScenarioArmTrain::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	cScenarioArm::UpdateCharacter(time_step);

	if (new_update)
	{
		RecordState(mCurrTuple.mStateEnd);
		RecordFlagsEnd(mCurrTuple);
		mCurrTuple.mReward = CalcReward();

		if (mEnableTraining)
		{
			if (mValidSample)
			{
				RecordTuple(mCurrTuple);
			}

			if (mNumTuples >= static_cast<int>(mTupleBuffer.size()))
			{
				Train();
			}
		}

		mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
		RecordAction(mCurrTuple.mAction);
		ClearFlags(mCurrTuple);
		RecordFlagsBeg(mCurrTuple);

		mValidSample = true;
		PrintInfo();
	}
}

void cScenarioArmTrain::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 2;
	out_max = 6;
	//out_min = 20;
	//out_max = 40;
}

void cScenarioArmTrain::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 6;
	out_max = 8;
	//out_min = 20;
	//out_max = 40;
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
	auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
	out_trainer = trainer;
}

void cScenarioArmTrain::InitTrainer()
{
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mInitInputOffsetScale = false;
	mTrainerParams.mNumInitSamples = 20000;
	//mTrainerParams.mFreezeTargetIters = 200;
	//mTrainerParams.mFreezeTargetIters = 5;
	mTrainerParams.mPGMode = cCaclaTrainer::ePGModeCacla;
	mTrainerParams.mPGAdvScale = 20;
	//mTrainerParams.mRewardMode = cTrainerInterface::eRewardModeAvg;

	mTrainer->Init(mTrainerParams);
	SetupScale();

	SetupActionBounds();
}

void cScenarioArmTrain::InitLearner()
{
	mTrainer->RequestLearner(mLearner);
	auto ctrl = GetController();

	cNeuralNet& net = ctrl->GetNet();
	mLearner->SetNet(&net);
	mLearner->Init();
}

void cScenarioArmTrain::SetupActionBounds()
{
	auto trainer = std::static_pointer_cast<cCaclaTrainer>(mTrainer);

	Eigen::VectorXd action_min;
	Eigen::VectorXd action_max;
	BuildActionBounds(action_min, action_max);
	trainer->SetActionBounds(action_min, action_max);
}

void cScenarioArmTrain::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	auto ctrl = GetController();
	ctrl->BuildActionBounds(out_min, out_max);
}

void cScenarioArmTrain::SetupScale()
{
	SetupActorScale();
	SetupCriticScale();
}

void cScenarioArmTrain::SetupActorScale()
{
	auto ctrl = GetController();
	int state_size = mTrainer->GetInputSize();
	int action_size = mTrainer->GetOutputSize();

	Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(state_size);
	Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(state_size);
	ctrl->BuildNNInputOffsetScale(input_offset, input_scale);
	mTrainer->SetActorInputOffsetScale(input_offset, input_scale);

	Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(action_size);
	ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);
	mTrainer->SetActorOutputOffsetScale(output_offset, output_scale);
}

void cScenarioArmTrain::SetupCriticScale()
{
	auto ctrl = GetController();
	int state_size = mTrainer->GetStateSize();
	int critic_input_size = mTrainer->GetCriticInputSize();
	int critic_output_size = mTrainer->GetCriticOutputSize();

	Eigen::VectorXd input_offset = Eigen::VectorXd::Zero(critic_input_size);
	Eigen::VectorXd input_scale = Eigen::VectorXd::Ones(critic_input_size);
	ctrl->BuildNNInputOffsetScale(input_offset, input_scale);

	assert(input_offset.size() == critic_input_size);
	assert(input_scale.size() == critic_input_size);
	mTrainer->SetCriticInputOffsetScale(input_offset, input_scale);

	Eigen::VectorXd critic_output_offset = -0.5 * Eigen::VectorXd::Ones(critic_output_size);
	//Eigen::VectorXd critic_output_offset = Eigen::VectorXd::Zero(critic_output_size);
	Eigen::VectorXd critic_output_scale = 2 * Eigen::VectorXd::Ones(critic_output_size);
	mTrainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
}

void cScenarioArmTrain::UpdatePolicy()
{
	const auto& learner_net = mLearner->GetNet();
	std::shared_ptr<cArmNNController> ctrl = GetController();
	if (ctrl != nullptr)
	{
		ctrl->CopyNet(*learner_net);
	}

	cCharController::tExpParams exp_params = ctrl->GetExpParams();
	exp_params.mRate = CalcExpRate();
	exp_params.mTemp = CalcExpTemp();
	ctrl->SetExpParams(exp_params);
}

void cScenarioArmTrain::Train()
{
	int iter = GetIter();
	int num_tuples = mLearner->GetNumTuples();
	printf("\nTraining Iter: %i\n", iter);
	printf("Num Tuples: %i\n", num_tuples);
	printf("Exp Rate: %.3f\n", CalcExpRate());
	printf("Exp Temp: %.3f\n", CalcExpTemp());

	mLearner->Train(mTupleBuffer);

	UpdatePolicy();
	mNumTuples = 0;
}

int cScenarioArmTrain::GetIter() const
{
	return mLearner->GetIter();
}

double cScenarioArmTrain::CalcExpRate() const
{
	int iters = GetIter();
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_rate = (1 - lerp) * mInitExpRate + lerp * mExpRate;
	return exp_rate;
}

double cScenarioArmTrain::CalcExpTemp() const
{
	int iters = GetIter();
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_rate = (1 - lerp) * mInitExpTemp + lerp * mExpTemp;
	return exp_rate;

	/*
	int iters = GetIter();
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_temp = (1 - lerp) / mInitExpTemp + lerp / mExpTemp;
	exp_temp = 1 / exp_temp;
	return exp_temp;
	*/
}

bool cScenarioArmTrain::CheckOffPolicy() const
{
	auto ctrl = GetController();
	bool off_poli = ctrl->IsOffPolicy();
	return off_poli;
}

std::shared_ptr<cArmNNController> cScenarioArmTrain::GetController() const
{
	const auto& ctrl = mChar->GetController();
	if (ctrl == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmNNController>(ctrl);
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

	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();

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
	printf("\n\n");
}