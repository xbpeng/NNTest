#include "ScenarioBallRL.h"

const int gTupleBufferSize = 16;
const int gTrainerPlaybackMemSize = 20000;

cScenarioBallRL::cScenarioBallRL()
{
	Clear();
	mEpsilon = 0.1;
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
	parser.ParseDouble("epsilon_greedy_rate", mEpsilon);
	parser.ParseDouble("ctrl_noise", mCtrlNoise);

	parser.ParseDouble("ground_height", mGroundParams.mHeight);
	parser.ParseDouble("ground_min_spacing", mGroundParams.mMinSpacing);
	parser.ParseDouble("ground_max_spacing", mGroundParams.mMaxSpacing);
	parser.ParseDouble("ground_min_box_size", mGroundParams.mMinBoxSize);
	parser.ParseDouble("ground_max_box_size", mGroundParams.mMaxBoxSize);

	parser.ParseDouble("ground_spacing_prob1", mGroundParams.mSpacingProb1);
	parser.ParseDouble("ground_min_spacing1", mGroundParams.mMinSpacing1);
	parser.ParseDouble("ground_max_spacing1", mGroundParams.mMaxSpacing1);

	parser.ParseInt("ground_num_boxes", mGroundParams.mNumBoxes);
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
	const cBallController& ctrl = mBall.GetController();
	ctrl.SaveNet(out_file);
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
	cBallController& ctrl = mBall.GetController();
	ctrl.SetGround(&mGround);
	ctrl.SetCtrlNoise(mCtrlNoise);

	if (mNetFile != "")
	{
		bool succ = ctrl.LoadNet(mNetFile);
		if (!succ)
		{
			printf("Failed to load network from %s\n", mNetFile.c_str());
		}
	}

	if (mModelFile != "")
	{
		ctrl.LoadModel(mModelFile);
	}
}

void cScenarioBallRL::UpdateGround()
{
	const double margin = 20;
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
	const cBallController& ctrl = mBall.GetController();
	return ctrl.GetStateSize();
}

int cScenarioBallRL::GetActionSize() const
{
	const cBallController& ctrl = mBall.GetController();
	return ctrl.GetActionSize();
}

void cScenarioBallRL::NewCycleUpdate()
{
	if (mEnableTraining)
	{
		// finish recording tuple from previous cycle
		RecordState(mCurrTuple.mStateEnd);
		mCurrTuple.mFail = CheckFail();
		mCurrTuple.mReward = CalcReward(mCurrTuple);

		// do something with the tuple
		if (!mFirstCycle)
		{
			mTupleBuffer[mNumTuples] = mCurrTuple;
			++mNumTuples;

			if (mNumTuples == static_cast<int>(mTupleBuffer.size()))
			{
				Train();
			}
		}

		double rand = cMathUtil::RandDouble(0, 1);
		if (rand < mEpsilon || mTrainer->GetIter() == 0)
		{
			ApplyRandAction();
		}

		// start recording new tuple
		mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
		RecordAction(mCurrTuple.mAction);

		mFirstCycle = false;
	}
}

void cScenarioBallRL::RecordState(Eigen::VectorXd& out_state) const
{
	const cBallController& ctrl = mBall.GetController();
	ctrl.RecordState(out_state);
}

void cScenarioBallRL::RecordAction(Eigen::VectorXd& out_action) const
{
	const cBallController& ctrl = mBall.GetController();
	ctrl.RecordAction(out_action);
}

double cScenarioBallRL::CalcReward(const tExpTuple& tuple) const
{
	double reward = 1;
	tVector ball_pos = mBall.GetPos();
	int penalty_idx = mGround.CheckPenaltyContact(ball_pos);
	bool contact = penalty_idx != -1;

	int action_idx = 0;
	tuple.mAction.maxCoeff(&action_idx);
	
	const cBallController& ctrl = mBall.GetController();
	const cBallController::tAction& action = ctrl.GetAction(action_idx);
	double dist = action.mDist;
	dist -= 0.5;

	reward /= (1 + dist * dist);

	double discount = mTrainer->GetDiscount();
	double norm = 1 / (1 - discount);

	reward *= 10 / norm;

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
	cBallController& ctrl = mBall.GetController();
	ctrl.ApplyRandAction();
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

	cQNetTrainer::tParams params;
	params.mNetFile = mNetFile;
	params.mSolverFile = mSolverFile;
	params.mPlaybackMemSize = gTrainerPlaybackMemSize;
	params.mPoolSize = 2; // double Q learning
	params.mNumInitSamples = 500;
	params.mFreezeTargetIters = 500;
	//params.mIntOutputFile = "output/intermediate/ball_int.h5";
	//params.mIntOutputIters = 10;
	trainer->Init(params);

	if (mModelFile != "")
	{
		trainer->LoadModel(mModelFile);
	}

	mTrainer = trainer;
}

void cScenarioBallRL::InitGround()
{
	mGround.Init(mGroundParams);
}

void cScenarioBallRL::Train()
{
	printf("\nTraining iter: %i\n", mTrainer->GetIter());

	const int num_steps = 1;

	mNumTuples = 0;
	mTrainer->AddTuples(mTupleBuffer);
	mTrainer->Train(num_steps);

	const cNeuralNet& trainer_net = mTrainer->GetNet();
	cBallController& ctrl = mBall.GetController();
	ctrl.CopyNet(trainer_net);
}