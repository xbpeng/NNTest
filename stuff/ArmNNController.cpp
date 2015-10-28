#include "ArmNNController.h"
#include "SimArm.h"

cArmNNController::cArmNNController()
{
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
		// hack
		mPoliState[0] = cMathUtil::RandDouble(-1, 1);
		mNet.Eval(mPoliState, mPoliAction);

		printf("Test %.5f, %.5f\n", mPoliState[0], mPoliAction[0]);
	}
}