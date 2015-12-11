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
		UpdateAction();
	}
}

void cBallController::Reset()
{
	mPhase = 0;
	mPosBeg = mBall.GetPos();
	mPosEnd = mBall.GetPos();
	mCurrAction = gActions[0];
	mGroundSamples = Eigen::VectorXd::Zero(gNumGroundSamples);
	mCurrActionIdx = 0;
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
	mNet.Clear();
	mNet.LoadNet(net_file);
	
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

void cBallController::UpdateAction()
{
	mOffPolicy = false;
	mPosBeg = mBall.GetPos();

	if (HasGround())
	{
		SampleGround(mGroundSamples);
	}

	int a = 0;
	if (HasNet())
	{
		a = CalcActionNet();
	}
	else
	{
		a = GetRandomAction();
		mOffPolicy = true;
	}
	
	ApplyAction(a);
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

int cBallController::CalcActionNet()
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

	return a;
}

int cBallController::GetRandomAction() const
{
	int a = cMathUtil::RandInt(0, gNumActions);
	return a;
}

cBallController::tAction cBallController::GetRandomActionDiscrete() const
{
	int a = GetRandomAction();
	return gActions[a];
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

cBallController::tAction cBallController::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == GetActionSize());
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
	out_action[mCurrActionIdx] = 1;
}

bool cBallController::IsOffPolicy() const
{
	return mOffPolicy;
}

void cBallController::ApplyRandAction()
{
	int a = GetRandomAction();
	ApplyAction(a);
	mOffPolicy = true;
	printf("rand action: %i\n", a);
}

void cBallController::CopyNet(const cNeuralNet& net)
{
	mNet.CopyModel(net);
}

void cBallController::SaveNet(const std::string& out_file) const
{
	mNet.OutputModel(out_file);
}

void cBallController::BuildState(Eigen::VectorXd& state) const
{
	state = mGroundSamples;
}

void cBallController::ApplyAction(int a)
{
	mCurrActionIdx = a;
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