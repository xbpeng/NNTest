#include "BallControllerMACEDPG.h"
#include "Ball.h"

const int gActionFragSize = 1;

cBallControllerMACEDPG::cBallControllerMACEDPG(cBall& ball) :
	cBallControllerDPG(ball)
{
	mNumActionFrags = 0;
}

cBallControllerMACEDPG::~cBallControllerMACEDPG()
{
}

int cBallControllerMACEDPG::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBallControllerMACEDPG::GetActionFragSize() const
{
	return gActionFragSize;
}

int cBallControllerMACEDPG::GetNetOutputSize() const
{
	return GetActorOutputSize();
}

bool cBallControllerMACEDPG::ValidCritic() const
{
	return mCriticNet.HasNet();
}

bool cBallControllerMACEDPG::LoadCriticNet(const std::string& net_file)
{
	bool succ = true;
	mCriticNet.Clear();
	mCriticNet.LoadNet(net_file);

	int input_size = mCriticNet.GetInputSize();
	int output_size = mCriticNet.GetOutputSize();
	int critic_input_size = GetCriticInputSize();
	int critic_output_size = GetCriticOutputSize();

	if (input_size != critic_input_size)
	{
		printf("Network input dimension does not match expected size (%i vs %i).\n", input_size, critic_input_size);
		succ = false;
	}

	if (output_size != critic_output_size)
	{
		printf("Network output dimension does not match expected size (%i vs %i).\n", output_size, critic_output_size);
		succ = false;
	}

	if (!succ)
	{
		mCriticNet.Clear();
		assert(false);
	}

	return succ;
}

void cBallControllerMACEDPG::LoadCriticModel(const std::string& model_file)
{
	mCriticNet.LoadModel(model_file);
}

const cNeuralNet& cBallControllerMACEDPG::GetActor() const
{
	return mNet;
}

cNeuralNet& cBallControllerMACEDPG::GetActor()
{
	return mNet;
}

const cNeuralNet& cBallControllerMACEDPG::GetCritic() const
{
	return mCriticNet;
}

cNeuralNet& cBallControllerMACEDPG::GetCritic()
{
	return mCriticNet;
}

int cBallControllerMACEDPG::GetActorInputSize() const
{
	return GetStateSize();
}

int cBallControllerMACEDPG::GetActorOutputSize() const
{
	return mNumActionFrags * gActionFragSize;
}

int cBallControllerMACEDPG::GetCriticInputSize() const
{
	return GetStateSize() + GetActionSize();
}

int cBallControllerMACEDPG::GetCriticOutputSize() const
{
	return 1;
}

void cBallControllerMACEDPG::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cBallControllerMACEDPG::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int num_actions = GetNumActionFrags();
	int action_size = GetActionFragSize();
	int output_size = num_actions * action_size;

	out_offset = Eigen::VectorXd::Ones(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	double min_dist = cBallController::gMinDist;
	double max_dist = cBallController::gMaxDist;
	double dist_scale = 2 / (max_dist - min_dist);

	double dist_steps = (max_dist - min_dist) / (num_actions + 1);
	for (int a = 0; a < num_actions; ++a)
	{
		double dist_offset = -(a + 1) * dist_steps;
		out_offset.segment(a * action_size, action_size) *= dist_offset;
		out_scale.segment(a * action_size, action_size) *= dist_scale;
	}
}

void cBallControllerMACEDPG::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetNumActionFrags();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

void cBallControllerMACEDPG::LoadNetIntern(const std::string& net_file)
{
	cBallControllerDPG::LoadNetIntern(net_file);
	UpdateFragParams();
}

void cBallControllerMACEDPG::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = num_outputs / gActionFragSize;
	mBoltzmannBuffer.resize(mNumActionFrags);
}