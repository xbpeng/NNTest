#include "BallController.h"
#include "Ball.h"

const double gDuration = 0.65;
const cBallController::tAction gActions[] =
{
	{ 0, 0.4, gInvalidLikelihood },
	{ 1, 0.6, gInvalidLikelihood },
	{ 2, 0.8, gInvalidLikelihood },
	{ 3, 1.1, gInvalidLikelihood },
	{ 4, 1.4, gInvalidLikelihood },
	{ 5, 1.7, gInvalidLikelihood },
	{ 6, 2.0, gInvalidLikelihood },
	{ 7, 2.3, gInvalidLikelihood },
};
const int gNumActions = sizeof(gActions) / sizeof(gActions[0]);
const int gNumGroundSamples = 100;
const double gGroundSampleDist = 8;

const double cBallController::gMinDist = 0.1;
const double cBallController::gMaxDist = 2.5;

const int gNumActionDistSamples = 2000;

cBallController::tExpParams::tExpParams()
{
	mRate = 0.2;
	mTemp = 0.5;
	mNoise = 0.25;
	mInternNoise = 1;
}

cBallController::tAction::tAction()
	: tAction(gInvalidIdx, 0, gInvalidLikelihood)
{
}

cBallController::tAction::tAction(int id, double dist, double likelihood)
{
	mID = id;
	mDist = dist;
	mParams = dist * Eigen::VectorXd::Ones(1);
	mLikelihood = likelihood;
}

cBallController::cBallController(cBall& ball) :
	mBall(ball)
{
	mOffPolicy = false;
	mGround = nullptr;
	mCtrlNoise = 0;

	mEnableExp = false;

	mRecordActionDist = false;

	Reset();
}

cBallController::~cBallController()
{
}

void cBallController::Update(double time_step)
{
	double delta_phase = CalcDeltaPhase();
	mPhase = std::min(1.0, mPhase + time_step * delta_phase);

	tVector ball_pos = CalcBallPos();
	mBall.SetPos(ball_pos);

	if (mPhase >= 1)
	{
		mPhase = 0;
	}

	if (IsNewCycle())
	{
		UpdateDistTravelled();
		UpdateAction();
	}
}

void cBallController::Reset()
{
	mPhase = 0;
	mPosBeg = mBall.GetPos();
	mPosEnd = mBall.GetPos();
	UpdateDistTravelled();
	mCurrAction = gActions[0];
	mGroundSamples = Eigen::VectorXd::Zero(gNumGroundSamples);
	mPoliState = Eigen::VectorXd::Zero(GetStateSize());
}

bool cBallController::IsNewCycle() const
{
	return mPhase == 0;
}

void cBallController::SetGround(cPenaltyGround* ground)
{
	mGround = ground;
}

void cBallController::SetCtrlNoise(double noise)
{
	mCtrlNoise = noise;
}

const Eigen::VectorXd& cBallController::GetGroundSamples() const
{
	return mGroundSamples;
}

tVector cBallController::GetGroundSamplePos(int s) const
{
	double dist = (gGroundSampleDist * s) / (gNumGroundSamples - 1);
	return tVector(dist, 0, 0, 0) + mPosBeg;
}

int cBallController::GetNumGroundSamples() const
{
	return gNumGroundSamples;
}

bool cBallController::LoadNet(const std::string& net_file)
{
	bool succ = true;
	LoadNetIntern(net_file);
	
	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int state_size = GetNetInputSize();
	int action_size = GetNetOutputSize();

	if (output_size != action_size)
	{
		printf("Network output dimension does not match number of actions (%i vs %i).\n", output_size, state_size);
		succ = false;
	}

	if (input_size != state_size)
	{
		printf("Network input dimension does not match state size (%i vs %i).\n", input_size, state_size);
		succ = false;
	}

	if (!succ)
	{
		mNet.Clear();
		assert(false);
	}

	return succ;
}

void cBallController::LoadModel(const std::string& model_file)
{
	mNet.LoadModel(model_file);
}

int cBallController::GetNetInputSize() const
{
	return GetStateSize();
}

int cBallController::GetNetOutputSize() const
{
	return GetActionSize();
}

void cBallController::LoadNetIntern(const std::string& net_file)
{
	mNet.Clear();
	mNet.LoadNet(net_file);
}

void cBallController::UpdateAction()
{
	mOffPolicy = true;
	mPosBeg = mBall.GetPos();

	if (HasGround())
	{
		SampleGround(mGroundSamples);
	}
	BuildPoliState(mPoliState);

	if (mRecordActionDist)
	{
		SampleActionDist(gNumActionDistSamples, mActionDistSamples);
	}
	
	tAction action;
	if (HasNet())
	{
		DecideAction(action);
	}
	else
	{
		GetRandomActionDiscrete(action);
	}
	
	ApplyAction(action);
}

double cBallController::CalcDeltaPhase() const
{
	double dur = gDuration;
	double delta_phase = 1 / dur;
	return delta_phase;
}

tVector cBallController::CalcBallPos() const
{
	tVector pos = mPosBeg;
	double dist = mPosEnd[0] - mPosBeg[0];

	double dur = gDuration;
	double g = gGravity[1];
	double max_h = -g * dur * dur / 4;

	double h = -4 * max_h * mPhase * mPhase + 4 * max_h * mPhase;

	pos[0] += dist * mPhase;
	pos[1] = h;

	return pos;
}

void cBallController::SampleGround(Eigen::VectorXd& out_samples) const
{
	for (int i = 0; i < gNumGroundSamples; ++i)
	{
		tVector sample_pos = GetGroundSamplePos(i);
		double h = mGround->SampleHeight(sample_pos);
		out_samples[i] = h;
	}
}

bool cBallController::HasGround() const
{
	return mGround != nullptr;
}

bool cBallController::HasNet() const
{
	return mNet.HasNet();
}

bool cBallController::ShouldExplore() const
{
	bool explore = false;
	if (mEnableExp)
	{
		double exp_rand = cMathUtil::RandDouble();
		explore = (exp_rand < mExpParams.mRate);
	}
	return explore;
}

void cBallController::DecideAction(tAction& out_action)
{
	bool explore = ShouldExplore();
	if (explore)
	{
		ExploreAction(out_action);
		mOffPolicy = true;
	}
	else
	{
		ExploitPolicy(out_action);
		mOffPolicy = false;
	}
}

void cBallController::ExploitPolicy(tAction& out_action)
{
	CalcActionNet(out_action);
}

void cBallController::ExploreAction(tAction& out_action)
{
	GetRandomAction(out_action);
}

void cBallController::CalcActionNet(tAction& out_action)
{
	mNet.Eval(mPoliState, out_action.mParams);

	int a = 0;
	out_action.mParams.maxCoeff(&a);
	out_action = gActions[a];
}

void cBallController::GetRandomAction(tAction& out_action)
{
	GetRandomActionDiscrete(out_action);
}

void cBallController::GetRandomActionDiscrete(tAction& out_action) const
{
	int a = cMathUtil::RandInt(0, gNumActions);
	out_action = gActions[a];
}

int cBallController::GetStateSize() const
{
	return gNumGroundSamples;
}

int cBallController::GetActionSize() const
{
	return gNumActions;
}

const cBallController::tAction& cBallController::GetAction(int a) const
{
	return gActions[a];
}

const cBallController::tAction& cBallController::GetCurrAction() const
{
	return mCurrAction;
}

double cBallController::GetActionLikelihood() const
{
	return mCurrAction.mLikelihood;
}

cBallController::tAction cBallController::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == gNumActions);
	int action_idx = 0;
	action_params.maxCoeff(&action_idx);
	return GetAction(action_idx);
}

void cBallController::RecordState(Eigen::VectorXd& out_state) const
{
	out_state = mPoliState;
}

void cBallController::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());
	out_action[mCurrAction.mID] = 1;
}

bool cBallController::IsOffPolicy() const
{
	return mOffPolicy;
}

void cBallController::ApplyRandAction()
{
	tAction action;
	GetRandomActionDiscrete(action);
	ApplyAction(action);
	mOffPolicy = true;
}

void cBallController::CopyNet(const cNeuralNet& net)
{
	mNet.CopyModel(net);
}

void cBallController::SaveNet(const std::string& out_file) const
{
	mNet.OutputModel(out_file);
}

double cBallController::GetDistTravelled() const
{
	return mDistTravelled;
}

void cBallController::SetExpParams(const tExpParams& params)
{
	mExpParams = params;
}

const cBallController::tExpParams& cBallController::GetExpParams() const
{
	return mExpParams;
}

void cBallController::EnableExp(bool enable)
{
	mEnableExp = enable;
}

void cBallController::BuildNNInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	int input_size = GetNetInputSize();
	out_types.resize(input_size);
	for (int i = 0; i < input_size; ++i)
	{
		out_types[i] = cNeuralNet::eOffsetScaleTypeNone;
	}
}

void cBallController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetActionSize();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

void cBallController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int action_size = GetActionSize();
	out_min = gMinDist * Eigen::VectorXd::Ones(action_size);
	out_max = gMaxDist * Eigen::VectorXd::Ones(action_size);
}

cNeuralNet& cBallController::GetNet()
{
	return mNet;
}

void cBallController::RecordActionDist(bool enable)
{
	mRecordActionDist = enable;
}

bool cBallController::RecordActionDist() const
{
	return mRecordActionDist;
}

const Eigen::MatrixXd& cBallController::GetActionDistSamples() const
{
	return mActionDistSamples;
}

const tVector& cBallController::GetPosBeg() const
{
	return mPosBeg;
}

void cBallController::BuildPoliState(Eigen::VectorXd& state) const
{
	state = Eigen::VectorXd::Zero(GetStateSize());
	if (HasGround())
	{
		state.segment(0, mGroundSamples.size()) = mGroundSamples;
	}
}

void cBallController::ApplyAction(int a)
{
	ApplyAction(gActions[a]);
}

void cBallController::ApplyAction(const tAction& action)
{
	mPosEnd = mPosBeg;

	double noise = cMathUtil::RandDouble(-1, 1);
	noise *= mCtrlNoise;

	double dist = action.mParams[0];
	dist = cMathUtil::Clamp(dist, gMinDist, gMaxDist);
	double noisy_dist = dist * (1 + noise);

	mPosEnd[0] += noisy_dist;
	mCurrAction = action;
	mCurrAction.mDist = dist;
	mCurrAction.mParams[0] = dist;

	/*
	printf("action: %i\n", mCurrAction.mID);
	for (int i = 0; i < mCurrAction.mParams.size(); ++i)
	{
		printf("%.4f ", mCurrAction.mParams[i]);
	}
	printf("\n");
	*/
}

void cBallController::UpdateDistTravelled()
{
	mDistTravelled = mPosEnd[0] - mPosBeg[0];
}

int cBallController::GetNumActionDistSamples() const
{
	return gNumActionDistSamples;
}

void cBallController::SampleActionDist(int num_samples, Eigen::MatrixXd& out_samples)
{
	out_samples.resize(GetNumActionDistSamples(), GetActionSize());

	tAction action;
	for (int i = 0; i < num_samples; ++i)
	{
		GetRandomActionDiscrete(action);
		out_samples = action.mParams;
	}
}