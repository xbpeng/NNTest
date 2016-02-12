#include "BallController.h"
#include "Ball.h"

const double gDuration = 0.65;
const cBallController::tAction gActions[] =
{
	{ 0, 0.4 },
	{ 1, 0.6 },
	{ 2, 0.8 },
	{ 3, 1.1 },
	{ 4, 1.4 },
	{ 5, 1.7 },
	{ 6, 2.0 },
	{ 7, 2.3 },
};
const int gNumActions = sizeof(gActions) / sizeof(gActions[0]);
const int gNumGroundSamples = 100;
const double gGroundSampleDist = 8;

const double cBallController::gMinDist = 0.1;
const double cBallController::gMaxDist = 2.5;

cBallController::cBallController(cBall& ball) :
	mBall(ball)
{
	mOffPolicy = false;
	mGround = nullptr;
	mCtrlNoise = 0;

	mEnableExp = false;
	mExpRate = 0.2;
	mExpTemp = 0.5;
	
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
		explore = (exp_rand < mExpRate);
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
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd action;
	mNet.Eval(state, action);

	int a = 0;
	action.maxCoeff(&a);

	printf("action: %i ", a);
	for (int i = 0; i < action.size(); ++i)
	{
		printf("%.4f ", action[i]);
	}
	printf("\n");

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

cBallController::tAction cBallController::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == gNumActions);
	int action_idx = 0;
	action_params.maxCoeff(&action_idx);
	return GetAction(action_idx);
}

void cBallController::RecordState(Eigen::VectorXd& out_state) const
{
	BuildState(out_state);
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
	printf("rand action: %i\n", action.mID);
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

void cBallController::SetExpRate(double rate)
{
	mExpRate = rate;
}

void cBallController::SetExpTemp(double temp)
{
	mExpTemp = temp;
}

void cBallController::EnableExp(bool enable)
{
	mEnableExp = enable;
}

void cBallController::BuildState(Eigen::VectorXd& state) const
{
	state = mGroundSamples;
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

	double dist = action.mDist;
	
	dist = cMathUtil::Clamp(dist, gMinDist, gMaxDist);
	dist *= 1 + noise;

	mPosEnd[0] += dist;
	mCurrAction = action;
}

void cBallController::UpdateDistTravelled()
{
	mDistTravelled = mPosEnd[0] - mPosBeg[0];
}