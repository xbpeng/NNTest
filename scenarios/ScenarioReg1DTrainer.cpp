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
}

void cScenarioReg1DTrainer::Clear()
{
	cScenarioReg1D::Clear();
}

void cScenarioReg1DTrainer::Update(double time_elapsed)
{
	cScenarioReg1D::Update(time_elapsed);
}

void cScenarioReg1DTrainer::AddPt(const tVector& pt)
{
	cScenarioReg1D::AddPt(pt);
}

void cScenarioReg1DTrainer::TrainNet()
{
	cScenarioReg1D::TrainNet();
}

std::string cScenarioReg1DTrainer::GetName() const
{
	return "Regression 1D Trainer";
}

void cScenarioReg1DTrainer::SetupTrainer()
{
}