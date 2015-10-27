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
	SetupNet();
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
	SetupNet();
}

void cScenarioReg1D::Clear()
{
	mSolverFile = "";
	mNetFile = "";
	mPts.clear();
	mNet.Clear();
	mEvalPts.clear();
	mNumEvalPts = 0;
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
}

const std::vector<tVector, Eigen::aligned_allocator<tVector>>& cScenarioReg1D::GetEvalPts() const
{
	return mEvalPts;
}

void cScenarioReg1D::TrainNet()
{
	if (GetNumPts() > 0)
	{
		cNeuralNet::tProblem prob;
		BuildProb(prob);

		Eigen::VectorXd mean;
		Eigen::VectorXd stdev;
		mNet.CalcMeanStdev(prob.mX, mean, stdev);
		mNet.SetMeanStdev(mean, stdev);

		mNet.Train(prob);
		EvalNet();
	}
}

std::string cScenarioReg1D::GetName() const
{
	return "Regression 1D";
}

void cScenarioReg1D::SetupNet()
{
	mNet.Clear();
	mNet.LoadNet(mNetFile);
	mNet.LoadSolver(mSolverFile);
}

void cScenarioReg1D::BuildProb(cNeuralNet::tProblem& out_prob) const
{
	int num_data = GetNumPts();
	const int x_size = mNet.GetInputSize();
	const int y_size = mNet.GetOutputSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = mPassesPerStep;

	for (int i = 0; i < num_data; ++i)
	{
		const tVector& pt = GetPt(i);
		auto curr_x = out_prob.mX.row(i);
		auto curr_y = out_prob.mY.row(i);

		curr_x(0) = pt(0);
		curr_y(0) = pt(1);
	}
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
		Eigen::VectorXd x(mNet.GetInputSize());
		Eigen::VectorXd y(mNet.GetOutputSize());

		for (int i = 0; i < mNumEvalPts; ++i)
		{
			x(0) = static_cast<double>(i) / (mNumEvalPts - 1) * (max_x - min_x) + min_x;
			mNet.Eval(x, y);

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