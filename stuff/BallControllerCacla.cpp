#include "BallControllerCacla.h"
#include "Ball.h"

cBallControllerCacla::cBallControllerCacla(cBall& ball) :
	cBallController(ball)
{
	mExpNoiseStd = 0.5;
}

cBallControllerCacla::~cBallControllerCacla()
{
}

int cBallControllerCacla::GetActionSize() const
{
	return 1;
}

void cBallControllerCacla::ApplyRandAction()
{
	tAction action;
	GetRandomActionCont(action);
	ApplyAction(action);
	mOffPolicy = true;
	printf("rand action: %.3f\n", action.mDist);
}

void cBallControllerCacla::RecordAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetActionSize());
	out_action[0] = mCurrAction.mDist;
}

cBallControllerCacla::tAction cBallControllerCacla::BuildActionFromParams(const Eigen::VectorXd& action_params) const
{
	assert(action_params.size() == 1);
	tAction action;
	action.mDist = action_params[0];
	return action;
}

bool cBallControllerCacla::ValidCritic() const
{
	return mCriticNet.HasNet();
}

bool cBallControllerCacla::LoadCriticNet(const std::string& net_file)
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

void cBallControllerCacla::LoadCriticModel(const std::string& model_file)
{
	mCriticNet.LoadModel(model_file);
}

const cNeuralNet& cBallControllerCacla::GetActor() const
{
	return mNet;
}

cNeuralNet& cBallControllerCacla::GetActor()
{
	return mNet;
}

const cNeuralNet& cBallControllerCacla::GetCritic() const
{
	return mCriticNet;
}

cNeuralNet& cBallControllerCacla::GetCritic()
{
	return mCriticNet;
}

void cBallControllerCacla::CopyActorNet(const cNeuralNet& net)
{
	mNet.CopyModel(net);
}

void cBallControllerCacla::CopyCriticNet(const cNeuralNet& net)
{
	mCriticNet.CopyModel(net);
}

int cBallControllerCacla::GetNetOutputSize() const
{
	return GetActorOutputSize();
}

int cBallControllerCacla::GetActorInputSize() const
{
	return GetStateSize();
}

int cBallControllerCacla::GetActorOutputSize() const
{
	return GetActionSize();
}

int cBallControllerCacla::GetCriticInputSize() const
{
	return GetStateSize();
}

int cBallControllerCacla::GetCriticOutputSize() const
{
	return 1;
}

void cBallControllerCacla::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	BuildActorOutputOffsetScale(out_offset, out_scale);
}

void cBallControllerCacla::BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetActionSize();
	double min_dist = gMinDist;
	double max_dist = gMaxDist;
	double offset = -0.5 * (max_dist + min_dist);
	double scale = 2 / (max_dist - min_dist);
	out_offset = offset * Eigen::VectorXd::Ones(output_size);
	out_scale = scale * Eigen::VectorXd::Ones(output_size);
}

void cBallControllerCacla::BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetCriticOutputSize();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

void cBallControllerCacla::BuildCriticInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	BuildNNInputOffsetScaleTypes(out_types);
}

void cBallControllerCacla::BuildActorInputOffsetScaleTypes(std::vector<cNeuralNet::eOffsetScaleType>& out_types) const
{
	BuildNNInputOffsetScaleTypes(out_types);
}

void cBallControllerCacla::BuildActionExpCovar(Eigen::VectorXd& out_covar) const
{
	out_covar = Eigen::VectorXd::Ones(GetActionSize());
	out_covar *= mExpNoiseStd * mExpNoiseStd;
}

void cBallControllerCacla::CalcActionNet(tAction& out_action)
{
	Eigen::VectorXd action;
	mNet.Eval(mPoliState, action);

	out_action.mDist = action[0];
	out_action.mLikelihood = gInvalidLikelihood;

	printf("action: %.5f\n", action[0], out_action.mDist);
}

void cBallControllerCacla::GetRandomAction(tAction& out_action)
{
	GetRandomActionCont(out_action);
}

void cBallControllerCacla::GetRandomActionCont(tAction& out_action)
{
	CalcActionNet(out_action);
	ApplyExpNoise(out_action);
}

void cBallControllerCacla::ApplyExpNoise(tAction& out_action)
{
	const double dist_mean = 0;
	const double dist_stdev = mExpNoiseStd;

	double old_dist = out_action.mDist;
	double rand_dist = cMathUtil::RandDoubleNorm(dist_mean, dist_stdev);
	double new_dist = old_dist + rand_dist;
	new_dist = cMathUtil::Clamp(new_dist, gMinDist, gMaxDist);
	rand_dist = new_dist - old_dist;

	double likelihood = cMathUtil::EvalGaussian(old_dist, dist_stdev * dist_stdev, new_dist);

	out_action.mDist = new_dist;
	out_action.mLikelihood = likelihood;
}

void cBallControllerCacla::SampleActionDist(int num_samples, Eigen::MatrixXd& out_samples)
{
	out_samples.resize(GetNumActionDistSamples(), GetActionSize());

	tAction action;
	for (int i = 0; i < num_samples; ++i)
	{
		GetRandomActionCont(action);
		out_samples(i, 0) = action.mDist;
	}
}