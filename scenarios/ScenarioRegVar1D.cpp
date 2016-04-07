#include "ScenarioRegVar1D.h"
#include "learning/VarNetTrainer.h"

cScenarioRegVar1D::cScenarioRegVar1D()
{
	mRand.Seed(0);
}

cScenarioRegVar1D::~cScenarioRegVar1D()
{
}

void cScenarioRegVar1D::Init()
{
	cScenarioReg1D::Init();
	GenPoints();
}

void cScenarioRegVar1D::Reset()
{
	cScenarioReg1D::Reset();
	GenPoints();
}

std::string cScenarioRegVar1D::GetName() const
{
	return "Regression Var 1D";
}

void cScenarioRegVar1D::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	out_trainer = std::shared_ptr<cVarNetTrainer>(new cVarNetTrainer());
}

void cScenarioRegVar1D::GenPoints()
{
	const int num_pts = 100;
	const double min_x = -1.5;
	const double max_x = 1.5;
	const double max_delta = 1;
	const double max_dy = 0.2;
	const double max_y = 2;

	double dy = 0;
	double y = 0;
	double stdev = 0.2;

	double dx = (max_x - min_x) / (num_pts - 1);
	for (int i = 0; i < num_pts; ++i)
	{
		double x = min_x + i * dx;

		double delta = mRand.RandDouble(0, max_delta * dx);
		
		double sign_rand = mRand.RandDouble(-1, 1);
		
		double dy_threshold = dy / max_dy;
		double y_threshold = y / max_y;

		double sign_threshold = 0;
		if (std::abs(dy_threshold) > std::abs(y_threshold))
		{
			sign_threshold = dy_threshold;
		}
		else
		{
			sign_threshold = y_threshold;
		}

		bool neg = sign_rand < sign_threshold;
		delta = (neg) ? -delta : delta;

		dy += delta;
		y += dy;

		//double curr_stdev = stdev;
		double curr_stdev = stdev * std::exp(-2 * x * x);
		double y_stdev = mRand.RandDoubleNorm(0, curr_stdev);
		double y1 = y +  y_stdev;

		AddPt(tVector(x, y1, 0, 0));
	}
}

tVector cScenarioRegVar1D::BuildPt(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const
{
	return tVector(x[0], y[0], y[1], 0);
}