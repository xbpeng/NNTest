#include "ScenarioReg1D.h"

cScenarioReg1D::cScenarioReg1D()
{
	Clear();
}

cScenarioReg1D::~cScenarioReg1D()
{
}

void cScenarioReg1D::Init()
{
	InitTrainer();
	InitLearner();
}

void cScenarioReg1D::ParseArgs(const cArgParser& parser)
{
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseInt("num_evals_pts", mNumEvalPts);
	parser.ParseInt("pases_per_step", mPassesPerStep);
}

void cScenarioReg1D::Reset()
{
	mPts.clear();
	mEvalPts.clear();
	mTrainer->Reset();
}

void cScenarioReg1D::Clear()
{
	mSolverFile = "";
	mNetFile = "";
	mPts.clear();
	mTrainer.reset();
	mEvalPts.clear();
	mNumEvalPts = 100;
	mPassesPerStep = 100;
}

void cScenarioReg1D::Update(double time_elapsed)
{
}

int cScenarioReg1D::GetNumPts() const
{
	return static_cast<int>(mPts.size());
}

const tVector& cScenarioReg1D::GetPt(int i) const
{
	return mPts[i];
}

void cScenarioReg1D::AddPt(const tVector& pt)
{
	mPts.push_back(pt);
	tExpTuple tuple;
	BuildTuple(pt, tuple);
	mTrainer->AddTuple(tuple);
}

const std::vector<tVector, Eigen::aligned_allocator<tVector>>& cScenarioReg1D::GetEvalPts() const
{
	return mEvalPts;
}

void cScenarioReg1D::TrainNet()
{
	mTrainer->Train();
	EvalNet();
}

std::string cScenarioReg1D::GetName() const
{
	return "Regression 1D";
}

void cScenarioReg1D::InitTrainer()
{
	mTrainer = std::shared_ptr<cNeuralNetTrainer>(new cNeuralNetTrainer());

	cNeuralNetTrainer::tParams trainer_params;
	trainer_params.mNetFile = mNetFile;
	trainer_params.mSolverFile = mSolverFile;
	trainer_params.mPlaybackMemSize = 10000;
	trainer_params.mPoolSize = 1;
	trainer_params.mNumInitSamples = 0;
	trainer_params.mInitInputOffsetScale = false;

	mTrainer->Init(trainer_params);
	SetupScale();
}

void cScenarioReg1D::InitLearner()
{
	mTrainer->RequestLearner(mLearner);
}

void cScenarioReg1D::SetupScale()
{
	int state_size = mTrainer->GetStateSize();
	int action_size = mTrainer->GetActionSize();
	
	Eigen::VectorXd offset = Eigen::VectorXd::Zero(state_size);
	Eigen::VectorXd scale = Eigen::VectorXd::Ones(state_size);
	mTrainer->SetInputOffsetScale(offset, scale);

	Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(action_size);
	Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(action_size);
	mTrainer->SetOutputOffsetScale(output_offset, output_scale);
}

void cScenarioReg1D::BuildTuple(const tVector& pt, tExpTuple& out_tuple) const
{
	int state_size = mTrainer->GetStateSize();
	int action_size = mTrainer->GetActionSize();

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

void cScenarioReg1D::EvalNet()
{
	const double pad = 0.2;
	if (GetNumPts() > 0)
	{
		double min_x = 0;
		double max_x = 0;
		FindMinMaxX(min_x, max_x);
		min_x -= pad;
		max_x += pad;

		mEvalPts.resize(mNumEvalPts);

		const auto& net = GetNet();
		Eigen::VectorXd x = Eigen::VectorXd::Zero(net->GetInputSize());
		Eigen::VectorXd y = Eigen::VectorXd::Zero(net->GetOutputSize());

		for (int i = 0; i < mNumEvalPts; ++i)
		{
			x(0) = static_cast<double>(i) / (mNumEvalPts - 1) * (max_x - min_x) + min_x;
			net->Eval(x, y);
			mEvalPts[i] = tVector(x(0), y(0), 0, 0);
		}
	}
	else
	{
		mEvalPts.clear();
	}
}

void cScenarioReg1D::FindMinMaxX(double& out_min_x, double& out_max_x) const
{
	int num_pts = GetNumPts();
	if (num_pts > 0)
	{
		out_min_x = std::numeric_limits<double>::infinity();
		out_max_x = -std::numeric_limits<double>::infinity();

		for (int i = 0; i < num_pts; ++i)
		{
			const tVector& pt = GetPt(i);
			out_min_x = std::min(pt(0), out_min_x);
			out_max_x = std::max(pt(0), out_max_x);
		}
	}
	else
	{
		out_min_x = 0;
		out_max_x = 0;
	}
}

const std::unique_ptr<cNeuralNet>& cScenarioReg1D::GetNet() const
{
	return mTrainer->GetNet();
}