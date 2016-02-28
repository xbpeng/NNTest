#include "ArmNNController.h"
#include "SimArm.h"

cArmNNController::cArmNNController()
{
	mExpRate = 0.2;
	mExpNoise = 0.2;
	mEnableExp = false;
}

cArmNNController::~cArmNNController()
{
}

void cArmNNController::Init(cSimCharacter* character)
{
	cArmController::Init(character);
}

void cArmNNController::Reset()
{
	cArmController::Reset();
}

void cArmNNController::Clear()
{
	cCharController::Clear();
	mNet.Clear();
}

bool cArmNNController::LoadNet(const std::string& net_file)
{
	bool succ = true;
	mNet.Clear();
	mNet.LoadNet(net_file);

	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int state_size = GetPoliStateSize();
	int action_size = GetPoliActionSize();

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

void cArmNNController::LoadModel(const std::string& model_file)
{
	mNet.LoadModel(model_file);
}

void cArmNNController::LoadScale(const std::string& scale_file)
{
	mNet.LoadScale(scale_file);
}

void cArmNNController::CopyNet(const cNeuralNet& net)
{
	mNet.CopyModel(net);
}

void cArmNNController::SaveNet(const std::string& out_file) const
{
	mNet.OutputModel(out_file);
}

bool cArmNNController::HasNet() const
{
	return mNet.HasNet();
}

void cArmNNController::UpdatePoliAction()
{
	if (HasNet())
	{
		mNet.Eval(mPoliState, mPoliAction);
	}
}

void cArmNNController::DecideAction()
{
	UpdatePoliAction();

	if (mEnableExp)
	{
		bool exp = cMathUtil::FlipCoin(mExpRate);
		if (exp)
		{
			ApplyExpNoise(mPoliAction);
		}
	}
}

void cArmNNController::ApplyExpNoise(Eigen::VectorXd& out_action) const
{
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	for (int i = 0; i < out_action.size(); ++i)
	{
		double noise = cMathUtil::RandDoubleNorm(0, mExpNoise);
		double scale = noise_scale[i];
		noise *= scale;
		out_action[i] += noise;
	}
}

void cArmNNController::FetchExpNoiseScale(Eigen::VectorXd& out_noise) const
{
	const Eigen::VectorXd& nn_output_scale = mNet.GetOutputScale();
	out_noise = nn_output_scale.cwiseInverse();
}

void cArmNNController::ApplyPoliAction(double time_step, const Eigen::VectorXd& action)
{
	cArmController::ApplyPoliAction(time_step, action);
}