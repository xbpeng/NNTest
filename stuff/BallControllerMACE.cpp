#include "BallControllerMACE.h"
#include "Ball.h"

cBallControllerMACE::cBallControllerMACE(cBall& ball) :
	cBallControllerACE(ball)
{
}

cBallControllerMACE::~cBallControllerMACE()
{
}


bool cBallControllerMACE::LoadCriticNet(const std::string& net_file)
{
	bool succ = true;
	mCriticNet.Clear();
	mCriticNet.LoadNet(net_file);

	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int state_size = GetNetInputSize();
	int action_size = GetCriticNetOutputSize();

	if (output_size != action_size)
	{
		printf("Network output dimension does not match number of actions (%i vs %i).\n", output_size, action_size);
		succ = false;
	}

	if (input_size != state_size)
	{
		printf("Network input dimension does not match state size (%i vs %i).\n", input_size, state_size);
		succ = false;
	}

	if (!succ)
	{
		mCriticNet.Clear();
		assert(false);
	}

	return succ;
}

void cBallControllerMACE::LoadCriticModel(const std::string& model_file)
{
	mCriticNet.LoadModel(model_file);
}

void cBallControllerMACE::LoadCriticScale(const std::string& scale_file)
{
	mCriticNet.LoadScale(scale_file);
}

void cBallControllerMACE::CopyCriticNet(const cNeuralNet& net)
{
	mCriticNet.CopyModel(net);
}

void cBallControllerMACE::SaveCriticNet(const std::string& out_file) const
{
	mCriticNet.OutputModel(out_file);
}

bool cBallControllerMACE::HasNet() const
{
	return mNet.HasNet() && mCriticNet.HasNet();
}

int cBallControllerMACE::GetNetOutputSize() const
{
	return mNumActionFrags * GetActionFragSize();
}

int cBallControllerMACE::GetCriticNetOutputSize() const
{
	return mNumActionFrags;
}

void cBallControllerMACE::SaveNet(const std::string& out_file) const
{
	std::string critic_filename = cMACETrainer::GetCriticFilename(out_file);
	mNet.OutputModel(critic_filename);
	mCriticNet.OutputModel(critic_filename);
}

void cBallControllerMACE::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize() / GetActionFragSize();
	mNumActionFrags = num_outputs;
}

cBallController::tAction cBallControllerMACE::CalcActionNetCont()
{
	Eigen::VectorXd state;
	BuildState(state);

	Eigen::VectorXd critic_y;
	mCriticNet.Eval(state, critic_y);

	Eigen::VectorXd actor_y;
	mNet.Eval(state, actor_y);

	int a = GetMaxFragIdx(critic_y);
	Eigen::VectorXd action_frag;
	GetFrag(actor_y, a, action_frag);

	tAction ball_action;
	ball_action.mID = a;
	ball_action.mDist = action_frag[0];
	printf("action: %i, %.5f\n", a, ball_action.mDist);

	return ball_action;
}

int cBallControllerMACE::GetMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragIdx(params);
}

double cBallControllerMACE::GetMaxFragVal(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragVal(params);
}

void cBallControllerMACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cMACETrainer::GetFrag(params, GetActionFragSize(), a_idx, out_action);
}

void cBallControllerMACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetFrag(frag, a_idx, GetActionFragSize(), out_params);
}