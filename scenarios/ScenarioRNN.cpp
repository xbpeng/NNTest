#include "ScenarioRNN.h"

cScenarioRNN::cScenarioRNN()
{
	Clear();
}

cScenarioRNN::~cScenarioRNN()
{
}

void cScenarioRNN::Init()
{
	InitTrainer();
	GenPoints();
}

void cScenarioRNN::ParseArgs(const cArgParser& parser)
{
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseInt("pases_per_step", mPassesPerStep);
}

void cScenarioRNN::Reset()
{
	mPts.clear();
	mEvalPts.clear();
	mTrainer->Reset();
	GenPoints();
}

void cScenarioRNN::Clear()
{
	mSolverFile = "";
	mNetFile = "";
	mPts.clear();
	mTrainer.reset();
	mEvalPts.clear();
	mPassesPerStep = 100;
}

void cScenarioRNN::Update(double time_elapsed)
{
}

int cScenarioRNN::GetNumPts() const
{
	return static_cast<int>(mPts.size());
}

const tVector& cScenarioRNN::GetPt(int i) const
{
	return mPts[i];
}

void cScenarioRNN::AddPt(const tVector& pt)
{
	mPts.push_back(pt);
	int num_pts = GetNumPts();

	if (num_pts > 1)
	{
		tExpTuple tuple;
		bool is_start = mPts.size() == 2;
		tVector prev_pt = mPts[num_pts - 2];

		BuildTuple(prev_pt, pt, is_start, tuple);
		mTrainer->AddTuple(tuple);
	}
}

const std::vector<tVector, Eigen::aligned_allocator<tVector>>& cScenarioRNN::GetEvalPts() const
{
	return mEvalPts;
}

void cScenarioRNN::TrainNet()
{
	for (int i = 0; i < mPassesPerStep; ++i)
	{
		mTrainer->Train();
	}
	
	EvalNet();
}

std::string cScenarioRNN::GetName() const
{
	return "Regression 1D";
}

void cScenarioRNN::InitTrainer()
{
	BuildTrainer(mTrainer);

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

void cScenarioRNN::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	out_trainer = std::shared_ptr<cRNNTrainer>(new cRNNTrainer());
}

void cScenarioRNN::SetupScale()
{
	int input_size = mTrainer->GetInputSize();
	int output_size = mTrainer->GetOutputSize();
	
	Eigen::VectorXd offset = Eigen::VectorXd::Zero(input_size);
	Eigen::VectorXd scale = Eigen::VectorXd::Ones(input_size);
	mTrainer->SetInputOffsetScale(offset, scale);

	Eigen::VectorXd output_offset = Eigen::VectorXd::Zero(output_size);
	Eigen::VectorXd output_scale = Eigen::VectorXd::Ones(output_size);
	mTrainer->SetOutputOffsetScale(output_offset, output_scale);
}

void cScenarioRNN::BuildTuple(const tVector& pt0, const tVector& pt1, bool is_start, tExpTuple& out_tuple) const
{
	int state_size = mTrainer->GetStateSize();
	int action_size = mTrainer->GetActionSize();

	out_tuple.mStateBeg.resize(state_size);
	out_tuple.mAction.resize(action_size);

	out_tuple.ClearFlags();
	out_tuple.SetFlag(is_start, cRNNTrainer::eFlagStart);

	out_tuple.mStateBeg[0] = pt0[1];
	out_tuple.mAction[0] = pt1[1];
	out_tuple.mStateEnd = out_tuple.mStateBeg;
}

void cScenarioRNN::EvalNet()
{
	const double pad = 0.2;
	int num_pts = GetNumPts();
	if (num_pts > 0)
	{
		double min_x = 0;
		double max_x = 0;
		FindMinMaxX(min_x, max_x);
		min_x -= pad;
		max_x += pad;

		mEvalPts.resize(num_pts);

		const auto& net = GetNet();
		Eigen::VectorXd x = Eigen::VectorXd::Zero(net->GetInputSize());
		Eigen::VectorXd y = Eigen::VectorXd::Zero(net->GetOutputSize());

		mEvalPts[0] = mPts[0];
		for (int i = 1; i < num_pts; ++i)
		{
			bool is_start = i == 1;
			const tVector& prev_pt = mPts[i - 1];
			const tVector& curr_pt = mPts[i];

			x[0] = prev_pt[1];
			net->Eval(x, is_start, y);

			mEvalPts[i] = tVector(curr_pt[0], y[0], 0, 0);
		}
	}
	else
	{
		mEvalPts.clear();
	}
}

void cScenarioRNN::FindMinMaxX(double& out_min_x, double& out_max_x) const
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

void cScenarioRNN::GenPoints()
{
	const int num_pts = 100;
	const double min_x = -1.5;
	const double max_x = 1.5;
	const double y_amp = 0.25;
	const double period = 1;

	for (int i = 0; i < num_pts; ++i)
	{
		double x = min_x + i * (max_x - min_x) / (num_pts - 1);
		double y = y_amp * std::sin(2 * M_PI / period * x);
		AddPt(tVector(x, y, 0, 0));
	}
}

cRecurrentNet* cScenarioRNN::GetNet() const
{
	return static_cast<cRecurrentNet*>((mTrainer->GetNet()).get());
}