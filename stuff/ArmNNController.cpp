#include "ArmNNController.h"
#include "SimArm.h"

cArmNNController::cArmNNController()
{
	mExpParams.mRate = 0.2;
	mExpParams.mNoise = 0.5;
	mEnableExp = false;
	mOffPolicy = false;
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
	LoadNetIntern(net_file);

	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int nn_input_size = GetNetInputSize();
	int nn_output_size = GetNetOutputSize();

	if (output_size != nn_output_size)
	{
		printf("Network output dimension does not match expected size (%i vs %i).\n", output_size, nn_output_size);
		succ = false;
	}

	if (input_size != nn_input_size)
	{
		printf("Network input dimension does not match expected size (%i vs %i).\n", input_size, nn_input_size);
		succ = false;
	}

	if (!succ)
	{
		mNet.Clear();
		assert(false);
	}

	return succ;
}

void cArmNNController::LoadNetIntern(const std::string& net_file)
{
	mNet.Clear();
	mNet.LoadNet(net_file);
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

cNeuralNet& cArmNNController::GetNet()
{
	return mNet;
}

bool cArmNNController::IsOffPolicy() const
{
	return mOffPolicy;
}

void cArmNNController::UpdatePoliAction()
{
	if (HasNet())
	{
		mNet.Eval(mPoliState, mPoliAction.mParams);
	}
}

void cArmNNController::DecideAction()
{
	mOffPolicy = false;
	UpdatePoliAction();

	if (mEnableExp)
	{
		bool exp = cMathUtil::FlipCoin(mExpParams.mRate);
		if (exp)
		{
			ApplyExpNoise(mPoliAction);
			mOffPolicy = true;
		}
	}
}

void cArmNNController::ApplyExpNoise(tAction& out_action) const
{
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	for (int i = 0; i < out_action.mParams.size(); ++i)
	{
		double noise = cMathUtil::RandDoubleNorm(0, mExpParams.mNoise);
		double scale = noise_scale[i];
		noise *= scale;
		out_action.mParams[i] += noise;
	}
}

void cArmNNController::FetchExpNoiseScale(Eigen::VectorXd& out_noise) const
{
	const Eigen::VectorXd& nn_output_scale = mNet.GetOutputScale();
	out_noise = nn_output_scale.cwiseInverse();
}

void cArmNNController::ApplyPoliAction(double time_step, const tAction& action)
{
	cArmController::ApplyPoliAction(time_step, action);
}

int cArmNNController::GetNetInputSize() const
{
	return GetPoliStateSize();
}

int cArmNNController::GetNetOutputSize() const
{
	return GetPoliActionSize();
}