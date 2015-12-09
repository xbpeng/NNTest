#include "ScenarioReg1DTrainer.h"

cScenarioReg1DTrainer::cScenarioReg1DTrainer()
{
}

cScenarioReg1DTrainer::~cScenarioReg1DTrainer()
{
}

void cScenarioReg1DTrainer::Init()
{
	cScenarioReg1D::Init();
	SetupTrainer();
}

void cScenarioReg1DTrainer::ParseArgs(const cArgParser& parser)
{
	cScenarioReg1D::ParseArgs(parser);
}

void cScenarioReg1DTrainer::Reset()
{
	cScenarioReg1D::Reset();
	mTrainer.Reset();
}

void cScenarioReg1DTrainer::Clear()
{
	cScenarioReg1D::Clear();
	mTrainer.Reset();
}

void cScenarioReg1DTrainer::Update(double time_elapsed)
{
	cScenarioReg1D::Update(time_elapsed);
}

void cScenarioReg1DTrainer::AddPt(const tVector& pt)
{
	cScenarioReg1D::AddPt(pt);
	tExpTuple tuple;
	BuildTuple(pt, tuple);
	mTrainer.AddTuple(tuple);
}

void cScenarioReg1DTrainer::TrainNet()
{
	if (GetNumPts() > 0)
	{
		mTrainer.Train();

		const auto& trainer_net = mTrainer.GetNet();
		mNet.CopyModel(*trainer_net.get());
		EvalNet();
	}
}

std::string cScenarioReg1DTrainer::GetName() const
{
	return "Regression 1D Trainer";
}

void cScenarioReg1DTrainer::BuildTuple(const tVector& pt, tExpTuple& out_tuple) const
{
	int state_size = mTrainer.GetStateSize();
	int action_size = mTrainer.GetActionSize();

	out_tuple.mStateBeg.resize(state_size);
	out_tuple.mAction.resize(action_size);

	for (int i = 0; i < state_size; ++i)
	{
		out_tuple.mStateBeg[i] = cMathUtil::RandDouble(-1, 1);
	}
	for (int i = 0; i < action_size; ++i)
	{
		out_tuple.mAction[i] = cMathUtil::RandDouble(-1, 1);
	}

	out_tuple.mStateBeg[0] = pt[0];
	out_tuple.mAction[0] = pt[1];
	out_tuple.mStateEnd = out_tuple.mStateBeg;
}

void cScenarioReg1DTrainer::SetupTrainer()
{
	cNeuralNetTrainer::tParams params;
	params.mNetFile = mNetFile;
	params.mSolverFile = mSolverFile;
	params.mPlaybackMemSize = 1000;
	params.mPoolSize = 1;
	params.mNumInitSamples = 0;
	params.mInitInputOffsetScale = false;
	mTrainer.Init(params);
}