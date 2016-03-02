#include "ScenarioArmTrain.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTupleBufferSize = 32;
const int gTrainerPlaybackMemSize = 500000; // 25000;

cScenarioArmTrain::cScenarioArmTrain()
{
	mEnableTraining = true;
	mPretrain = false;
	mValidSample = false;

	mExpRate = 0.2;
	mInitExpRate = 1;
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
	InitTupleBuffer();
	UpdatePolicy();
	mValidSample = false;

	auto controller = GetController();
	controller->EnableExp(true);
	
	// hack
	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	int num_samples = 200;
	double min_theta = -M_PI;
	double max_theta = M_PI;
	int state_size = trainer->GetStateSize();
	int action_size = trainer->GetActionSize();
	
	tExpTuple tuple;
	tuple.mStateBeg = Eigen::VectorXd::Zero(state_size);
	tuple.mAction = Eigen::VectorXd::Zero(action_size);

	FILE* critic_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/critic_vals.txt", "w");
	FILE* actor_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/actor_data.txt", "w");
	FILE* dpg_f = cFileUtil::OpenFile("scripts/dpg_arm_plot/dpg_data.txt", "w");

	for (int i = 0; i < num_samples; ++i)
	{
		double curr_theta = (max_theta - min_theta) * i / (num_samples - 1.0) + min_theta;
		double chain_len = mChar->CalcJointChainLength(mChar->GetNumJoints() - 1);
		tMatrix rot_mat = cMathUtil::RotateMat(tVector(0, 0, 1, 0), curr_theta);
		tVector delta = rot_mat * tVector(chain_len, 0, 0, 0);

		//tuple.mStateBeg[2] = curr_theta;
		tuple.mStateBeg[4] = delta[0];
		tuple.mStateBeg[5] = delta[1];

		Eigen::VectorXd actor_y;
		trainer->EvalActor(tuple, actor_y);
		tuple.mAction = actor_y;

		Eigen::VectorXd critic_y;
		trainer->EvalCritic(tuple, critic_y);
		
		Eigen::VectorXd dpg_y;
		trainer->CalcDPG(tuple, dpg_y);

		fprintf(critic_f, "%.5f, %.5f\n", curr_theta, critic_y[0]);

		fprintf(actor_f, "%.5f", curr_theta);
		for (int j = 0; j < actor_y.size(); ++j)
		{
			fprintf(actor_f, ", %.5f", actor_y[j]);
		}
		fprintf(actor_f, "\n");

		fprintf(dpg_f, "%.5f", curr_theta);
		for (int j = 0; j < dpg_y.size(); ++j)
		{
			fprintf(dpg_f, ", %.5f", dpg_y[j]);
		}
		fprintf(dpg_f, "\n");
	}
	cFileUtil::CloseFile(actor_f);
	cFileUtil::CloseFile(dpg_f);
	cFileUtil::CloseFile(critic_f);
}

void cScenarioArmTrain::ParseArgs(const cArgParser& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser.ParseBool("arm_pretrain", mPretrain);
	parser.ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser.ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);
	parser.ParseDouble("exp_rate", mExpRate);
	parser.ParseDouble("init_exp_rate", mInitExpRate);
	parser.ParseInt("num_exp_anneal_iters", mNumAnnealIters);
	
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
	double record_rate = 0.1;
	if (cMathUtil::FlipCoin(record_rate))
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
	double tar_w = 0;// 0.8;
	double pose_w = 0.8; // 0.1
	double vel_w = 0.2; // 0.1

	int end_id = GetEndEffectorID();
	const tVector& tar_pos = GetTargetPos();
	tVector end_pos = mChar->CalcJointPos(end_id);

	tVector delta = mTargetPos - end_pos;
	double gamma = 1;
	double tar_reward = exp(-gamma * delta.squaredNorm());
	//double tar_reward = 1 / (1 + delta.squaredNorm());

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);
	double pose_reward = 1 / (1 + pose.squaredNorm());
	double vel_reward = 1 / (1 + 0.02 * vel.squaredNorm());

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

		if (mEnableTraining && (mNumTuples >= static_cast<int>(mTupleBuffer.size())))
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

		PrintInfo();
	}
}

void cScenarioArmTrain::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 20;
	out_max = 40;
}

void cScenarioArmTrain::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 20;
	out_max = 40;
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
	trainer->SetDPGReg(0.0001);
	trainer->SetPretrainIters(1000);
	trainer->SetQDiff(4);
	out_trainer = trainer;
}

void cScenarioArmTrain::InitTrainer()
{
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mNumInitSamples = 20000;
	mTrainerParams.mInitInputOffsetScale = false;
	//mTrainerParams.mDiscount = 0.9;

	mTrainer->Init(mTrainerParams);
	SetupScale();
	SetupActionBounds();

	auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
	if (mCriticModelFile != "")
	{
		trainer->LoadCriticModel(mCriticModelFile);
	}

	if (mActorModelFile != "")
	{
		trainer->LoadActorModel(mActorModelFile);
	}

	//if (mScaleFile != "")
	//{
		//mTrainer->LoadScale(mScaleFile);
	//}
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

	double exp_rate = CalcExpRate();
	ctrl->SetExpRate(exp_rate);
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
	printf("Exp Rate: %.3f\n", CalcExpRate());

	mTrainer->AddTuples(mTupleBuffer);

	mTrainer->Train();
	UpdatePolicy();
	mNumTuples = 0;
}

int cScenarioArmTrain::GetIter() const
{
	return mTrainer->GetIter();
}

double cScenarioArmTrain::CalcExpRate() const
{
	int iters = GetIter();
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_rate = (1 - lerp) * mInitExpRate + lerp * mExpRate;
	return exp_rate;
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


	if (!mEnableTraining)
	{
		Eigen::VectorXd state;
		Eigen::VectorXd action;
		RecordState(state);
		RecordAction(action);
		tExpTuple tuple;
		tuple.mStateBeg = state;
		tuple.mAction = action;

		auto trainer = std::static_pointer_cast<cDPGTrainer>(mTrainer);
		Eigen::VectorXd dpg;
		trainer->CalcDPG(tuple, dpg);

		printf("DPG:\n");
		for (int i = 0; i < dpg.size(); ++i)
		{
			printf("%.5f\t", dpg[i]);
		}
		printf("\n");
	}

	printf("\n");
}