#include "ScenarioRNN.h"

const double gDefaultInput = 0;

cScenarioRNN::cScenarioRNN()
{
	mPrevID = gInvalidIdx;
}

cScenarioRNN::~cScenarioRNN()
{
}

void cScenarioRNN::Init()
{
	cScenarioReg1D::Init();
	mPrevID = gInvalidIdx;
}

void cScenarioRNN::Reset()
{
	cScenarioReg1D::Reset();
	mPrevID = gInvalidIdx;
}

void cScenarioRNN::AddPt(const tVector& pt)
{
	bool is_start = mPts.size() == 0;
	AddPt(pt, is_start);
}

void cScenarioRNN::AddPt(const tVector& pt, bool is_start)
{
	tVector curr_pt = pt;
	curr_pt[3] = (is_start) ? 1 : 0;
	mPts.push_back(curr_pt);
	int num_pts = GetNumPts();

	tExpTuple tuple;
	BuildTuple(curr_pt, is_start, tuple);
	int prev_id = (is_start) ? gInvalidIdx : mPrevID;

	auto trainer = GetRNNTrainer();
	int curr_id = trainer->AddTuple(tuple, prev_id);
	mPrevID = curr_id;

	printf("Is start: %s\n", (is_start) ? "true" : "false");
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

		const auto& net = GetRNN();
		Eigen::VectorXd x = Eigen::VectorXd::Zero(net->GetInputSize());
		Eigen::VectorXd y = Eigen::VectorXd::Zero(net->GetOutputSize());

		for (int i = 0; i < num_pts; ++i)
		{
			int idx = i;
			const tVector& curr_pt = mPts[idx];
			bool is_start = (curr_pt[3] != 0);
			x[0] = gDefaultInput;
			net->Eval(x, is_start, y);

			mEvalPts[i] = tVector(curr_pt[0], y[0], 0, (is_start) ? 1 : 0);
		}
	}
	else
	{
		mEvalPts.clear();
	}
}

void cScenarioRNN::GenPoints()
{
	int num_seqs = 8;
	int max_pts = 100;
	for (int i = 0; i < num_seqs; ++i)
	{
		double len_lerp = i / (num_seqs - 1.0);
		len_lerp *= 0.25;
		len_lerp = 1 - len_lerp;

		const int num_pts = max_pts * len_lerp;
		const double min_x = -1.5;
		const double max_x = 1.5;
		const double y_amp = 0.25;
		const double period = 1;

		double x_bias = 0;// i / (num_seqs - 1.0) * 0.01;
		double curr_y = 0;
		for (int i = 0; i < num_pts; ++i)
		{
			double x = min_x + i * (max_x - min_x) / (max_pts - 1);
			x += x_bias;

			double y = y_amp * std::sin(2 * M_PI / period * x);
			//y += 0.1 * std::sin(4 * M_PI / period * x);
			//y += 0.1 * std::sin(8 * M_PI / period * x);
			//y = (i % 9 < 6) ? -0.2 : 0.2;
			y *= i / (num_pts - 1.0);
			//y = curr_y;
			//curr_y += cMathUtil::RandDoubleNorm(0, 0.05);
			// y = 0.5 * x * x - 0.3;

			//int idx = i % 20;
			//y = 0.2 * idx / 19;

			//y = i / (num_pts - 1.0);
			//y -= 0.5;
			//y = std::abs(y);

			bool is_start = (i == 0);
			AddPt(tVector(x, y, 0, 0), is_start);
		}
	}
}

cRecurrentNet* cScenarioRNN::GetRNN() const
{
	return static_cast<cRecurrentNet*>((mTrainer->GetNet()).get());
}

std::shared_ptr<cRNNTrainer> cScenarioRNN::GetRNNTrainer() const
{
	return std::static_pointer_cast<cRNNTrainer>(mTrainer);
}