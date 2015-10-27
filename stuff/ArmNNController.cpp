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
	int state_size = GetStateSize();
	int action_size = GetActionSize();

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

int cArmNNController::GetStateSize() const
{
	int num_dof = mChar->GetNumDof();
	int root_size = mChar->GetParamSize(mChar->GetRootID());
	int pose_dim = num_dof - root_size;
	int state_size = 2 * pose_dim;
	return state_size;
}

int cArmNNController::GetActionSize() const
{
	return GetControlDim();
}

void cArmNNController::UpdateContolForce()
{
}