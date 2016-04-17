#include "ScenarioRNN.h"

const double gDefaultInput = 1;

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

	tExpTuple tuple;
	bool is_start = mPts.size() == 1;
	tVector curr_pt = mPts[num_pts - 1];

	BuildTuple(curr_pt, is_start, tuple);
	mTrainer->AddTuple(tuple);
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
	return "Regression RNN";
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

void cScenarioRNN::BuildTuple(const tVector& pt, bool is_start, tExpTuple& out_tuple) const
{
	int state_size = mTrainer->GetStateSize();
	int action_size = mTrainer->GetActionSize();

	out_tuple.mStateBeg.resize(state_size);
	out_tuple.mAction.resize(action_size);

	out_tuple.ClearFlags();
	out_tuple.SetFlag(is_start, cRNNTrainer::eFlagStart);

	out_tuple.mStateBeg[0] = gDefaultInput;
	out_tuple.mAction[0] = pt[1];
}

void cScenarioRNN::EvalNet()
{
	const double pad = 0.2;
	int num_pts = GetNumPts();
	if (num_pts > 0)
	{
		mEvalPts.resize(num_pts);

		const auto& net = GetNet();
		Eigen::VectorXd x = Eigen::VectorXd::Zero(net->GetInputSize());
		Eigen::VectorXd y = Eigen::VectorXd::Zero(net->GetOutputSize());

		for (int i = 0; i < num_pts; ++i)
		{
			bool is_start = (i == 0);
			const tVector& curr_pt = mPts[i];

			x[0] = gDefaultInput;
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

	double curr_y = 0;
	for (int i = 0; i < num_pts; ++i)
	{
		double x = min_x + i * (max_x - min_x) / (num_pts - 1);
		double y = y_amp * std::sin(2 * M_PI / period * x);
		y += 0.1 * std::sin(4 * M_PI / period * x);
		y += 0.1 * std::sin(8 * M_PI / period * x);
		//y = (i % 9 < 6) ? -0.2 : 0.2;
		//y *= 1 - i / (num_pts - 1.0);
		//y = curr_y;
		//curr_y += cMathUtil::RandDoubleNorm(0, 0.05);
		// y = 0.5 * x * x - 0.3;

		//int idx = i % 20;
		//y = 0.2 * idx / 19;

		//y = i / (num_pts - 1.0);
		//y -= 0.5;
		//y = std::abs(y);

		AddPt(tVector(x, y, 0, 0));
	}
}

cRecurrentNet* cScenarioRNN::GetNet() const
{
	return static_cast<cRecurrentNet*>((mTrainer->GetNet()).get());
}