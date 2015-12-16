#include "ScenarioBallRL.h"

const int gTupleBufferSize = 32;
const int gTrainerPlaybackMemSize = 50000;

cScenarioBallRL::cScenarioBallRL()
{
	Clear();
	mInitExpRate = 1;
	mInitExpTemp = 20;
	mExpRate = 0.1;
	mExpTemp = 0.025;
	mNumExpAnnealIters = 5000;
	mCtrlNoise = 0;
	mEnableTraining = true;
}

cScenarioBallRL::~cScenarioBallRL()
{
}

void cScenarioBallRL::Init()
{
	SetupController();
	mCurrTuple = tExpTuple(GetStateSize(), GetActionSize());
	InitTupleBuffer();
	InitTrainer();
	InitGround();
	Reset();
}

void cScenarioBallRL::ParseArgs(const cArgParser& parser)
{
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseString("model_file", mModelFile);
	parser.ParseDouble("ctrl_noise", mCtrlNoise);

	parser.ParseDouble("exp_rate", mExpRate);
	parser.ParseDouble("exp_temp", mExpTemp);
	parser.ParseInt("num_exp_anneal_iters", mNumExpAnnealIters);

	parser.ParseDouble("ground_height", mGroundParams.mHeight);
	parser.ParseDouble("ground_min_spacing", mGroundParams.mMinSpacing);
	parser.ParseDouble("ground_max_spacing", mGroundParams.mMaxSpacing);
	parser.ParseDouble("ground_min_box_size", mGroundParams.mMinBoxSize);
	parser.ParseDouble("ground_max_box_size", mGroundParams.mMaxBoxSize);

	parser.ParseDouble("ground_spacing_prob1", mGroundParams.mSpacingProb1);
	parser.ParseDouble("ground_min_spacing1", mGroundParams.mMinSpacing1);
	parser.ParseDouble("ground_max_spacing1", mGroundParams.mMaxSpacing1);

	parser.ParseInt("ground_num_boxes", mGroundParams.mNumBoxes);

	parser.ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser.ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);
}

void cScenarioBallRL::Reset()
{
	mBall.Reset();
	mGround.Reset();
	mFirstCycle = true;
	mNumTuples = 0;
}

void cScenarioBallRL::Clear()
{
	
}

void cScenarioBallRL::Update(double time_elapsed)
{
	if (time_elapsed > 0)
	{
		mBall.Update(time_elapsed);
		UpdateGround();

		if (mBall.IsNewCycle())
		{
			NewCycleUpdate();
		}
	}
}

const cPenaltyGround& cScenarioBallRL::GetGround() const
{
	return mGround;
}

const tVector& cScenarioBallRL::GetBallPos() const
{
	return mBall.GetPos();
}

void cScenarioBallRL::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;
	auto ctrl = mBall.GetController();
	ctrl->EnableExp(mEnableTraining);
}

bool cScenarioBallRL::EnableTraining() const
{
	return mEnableTraining;
}

double cScenarioBallRL::GetSuccRate() const
{
	double succ_rate = mGround.GetSuccessRate();
	return succ_rate;
}

void cScenarioBallRL::SaveNet(const std::string& out_file) const
{
	const auto& ctrl = mBall.GetController();
	ctrl->SaveNet(out_file);
}

const cBall& cScenarioBallRL::GetBall() const
{
	return mBall;
}

std::string cScenarioBallRL::GetName() const
{
	return "Ball RL";
}

void cScenarioBallRL::SetupController()
{
	std::shared_ptr<cBallController> ctrl;
	BuildController(ctrl);
	ctrl->SetGround(&mGround);
	ctrl->SetCtrlNoise(mCtrlNoise);

	if (mNetFile != "")
	{
		bool succ = ctrl->LoadNet(mNetFile);
		if (!succ)
		{
			printf("Failed to load network from %s\n", mNetFile.c_str());
		}
	}

	if (mModelFile != "")
	{
		ctrl->LoadModel(mModelFile);
	}

	mBall.SetController(ctrl);
	ctrl->EnableExp(true);
}

void cScenarioBallRL::BuildController(std::shared_ptr<cBallController>& out_ctrl)
{
	out_ctrl = std::shared_ptr<cBallController>(new cBallController(mBall));
}

void cScenarioBallRL::UpdateGround()
{
	const double margin = 10;
	tVector ball_pos = mBall.GetPos();
	mGround.Update(ball_pos[0] - margin, ball_pos[0] + margin);

	if (mBall.IsNewCycle())
	{
		int penalty_idx = mGround.CheckPenaltyContact(ball_pos);
		bool contact = penalty_idx != -1;
		if (contact)
		{
			mGround.SetHit(penalty_idx, true);
		}
	}
}

int cScenarioBallRL::GetStateSize() const
{
	const auto& ctrl = mBall.GetController();
	return ctrl->GetStateSize();
}

int cScenarioBallRL::GetActionSize() const
{
	const auto& ctrl = mBall.GetController();
	return ctrl->GetActionSize();
}

void cScenarioBallRL::NewCycleUpdate()
{
	if (EnableTraining())
	{
		// finish recording tuple from previous cycle
		RecordState(mCurrTuple.mStateEnd);
		RecordEndFlags(mCurrTuple);
		mCurrTuple.mReward = CalcReward(mCurrTuple);

		// do something with the tuple
		if (!mFirstCycle)
		{
			mTupleBuffer[mNumTuples] = mCurrTuple;
			++mNumTuples;

			if (mNumTuples == static_cast<int>(mTupleBuffer.size()))
			{
				Train();

				double exp_rate = GetExpRate();
				double exp_temp = GetExpTemp();
				auto& ctrl = mBall.GetController();
				ctrl->SetExpRate(exp_rate);
				ctrl->SetExpTemp(exp_temp);

				printf("\n");
				printf("Exp Rate: %.3f\n", exp_rate);
				printf("Exp Temp: %.3f\n", exp_temp);
			}
		}
		
		// start recording new tuple
		mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
		RecordAction(mCurrTuple.mAction);
		ClearFlags(mCurrTuple);
		RecordBegFlags(mCurrTuple);

		mFirstCycle = false;
	}
}

void cScenarioBallRL::ClearFlags(tExpTuple& out_tuple) const
{
	out_tuple.mFlags = 0;
}

void cScenarioBallRL::RecordBegFlags(tExpTuple& out_tuple) const
{
}

void cScenarioBallRL::RecordEndFlags(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cQNetTrainer::eFlagFail);
}

void cScenarioBallRL::RecordState(Eigen::VectorXd& out_state) const
{
	const auto& ctrl = mBall.GetController();
	ctrl->RecordState(out_state);
}

void cScenarioBallRL::RecordAction(Eigen::VectorXd& out_action) const
{
	const auto& ctrl = mBall.GetController();
	ctrl->RecordAction(out_action);
}

double cScenarioBallRL::CalcReward(const tExpTuple& tuple) const
{
	double reward = 0;
	tVector ball_pos = mBall.GetPos();
	int penalty_idx = mGround.CheckPenaltyContact(ball_pos);
	bool contact = penalty_idx != -1;

	const auto& ctrl = mBall.GetController();
	double dist = ctrl->GetDistTravelled();
	
	const double tar_dist = 0.5;
	double dist_err = dist - tar_dist;
	
	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double gamma = (dist_err >= 0) ? (max_dist - tar_dist) : (tar_dist - min_dist);
	gamma /= 1.5;

	gamma = 1 / gamma;
	gamma *= gamma;

	dist_err *= dist_err;
	double dist_reward = std::exp(-gamma * dist_err);
	reward = dist_reward;

	if (contact)
	{
		reward = 0;
	}
	return reward;
}

bool cScenarioBallRL::CheckFail() const
{
	tVector ball_pos = mBall.GetPos();
	int penalty_idx = mGround.CheckPenaltyContact(ball_pos);
	bool fail = penalty_idx != -1;
	return fail;
}

void cScenarioBallRL::ApplyRandAction()
{
	auto& ctrl = mBall.GetController();
	ctrl->ApplyRandAction();
}

void cScenarioBallRL::InitTupleBuffer()
{
	mTupleBuffer.resize(gTupleBufferSize);
	for (int i = 0; i < gTupleBufferSize; ++i)
	{
		mTupleBuffer[i] = tExpTuple(GetStateSize(), GetActionSize());
	}
}

void cScenarioBallRL::InitTrainer()
{
	std::shared_ptr<cQNetTrainer> trainer = std::shared_ptr<cQNetTrainer>(new cQNetTrainer());

	mTrainerParams.mNetFile = mNetFile;
	mTrainerParams.mSolverFile = mSolverFile;
	mTrainerParams.mPlaybackMemSize = gTrainerPlaybackMemSize;
	mTrainerParams.mPoolSize = 2; // double Q learning
	mTrainerParams.mNumInitSamples = 10000;
	mTrainerParams.mNumStepsPerIter = 1;
	//mTrainerParams.mNumInitSamples = 5;
	//mTrainerParams.mFreezeTargetIters = 500;

	trainer->Init(mTrainerParams);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	mTrainer = trainer;
	SetupTrainerOutputOffsetScale();
}

void cScenarioBallRL::InitGround()
{
	mGround.Init(mGroundParams);
}

void cScenarioBallRL::SetupTrainerOutputOffsetScale()
{
	Eigen::VectorXd output_offset;
	Eigen::VectorXd output_scale;
	BuildOutputOffsetScale(mTrainer, output_offset, output_scale);
	mTrainer->SetOutputOffsetScale(output_offset, output_scale);
}

void cScenarioBallRL::BuildOutputOffsetScale(const std::shared_ptr<cNeuralNetTrainer>& trainer, 
											Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = trainer->GetOutputSize();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

int cScenarioBallRL::GetIter() const
{
	return mTrainer->GetIter();
}

void cScenarioBallRL::Train()
{
	printf("\nTraining iter: %i\n", GetIter());
	printf("Num Tuples: %i\n", mTrainer->GetNumTuples());

	mNumTuples = 0;
	mTrainer->AddTuples(mTupleBuffer);
	mTrainer->Train();

	const auto& trainer_net = mTrainer->GetNet();
	auto& ctrl = mBall.GetController();
	ctrl->CopyNet(*trainer_net.get());
}

double cScenarioBallRL::GetExpRate() const
{
	int iter = GetIter();
	double eps = 1 - static_cast<double>(iter) / mNumExpAnnealIters;
	eps = cMathUtil::Clamp(eps, 0.0, 1.0);
	eps = eps * (mInitExpRate - mExpRate) + mExpRate;
	return eps;
}

double cScenarioBallRL::GetExpTemp() const
{
	int iter = GetIter();
	double temp = 1 - static_cast<double>(iter) / mNumExpAnnealIters;
	temp = cMathUtil::Clamp(temp, 0.0, 1.0);
	temp = temp * (mInitExpTemp - mExpTemp) + mExpTemp;
	return temp;
}