#include "ScenarioRNN.h"

const double gDefaultInput = 1;

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
	mPts.push_back(pt);
	int num_pts = GetNumPts();

	tExpTuple tuple;
	tVector curr_pt = mPts[num_pts - 1];

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

cRecurrentNet* cScenarioRNN::GetRNN() const
{
	return static_cast<cRecurrentNet*>((mTrainer->GetNet()).get());
}

std::shared_ptr<cRNNTrainer> cScenarioRNN::GetRNNTrainer() const
{
	return std::static_pointer_cast<cRNNTrainer>(mTrainer);
}