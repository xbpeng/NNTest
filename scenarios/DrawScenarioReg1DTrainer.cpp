#include "DrawScenarioReg1DTrainer.h"
#include "ScenarioReg1DTrainer.h"


cDrawScenarioReg1DTrainer::cDrawScenarioReg1DTrainer(cCamera& cam)
	: cDrawScenarioReg1D(cam)
{
}

cDrawScenarioReg1DTrainer::~cDrawScenarioReg1DTrainer()
{
}

void cDrawScenarioReg1DTrainer::BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioReg1D>(new cScenarioReg1DTrainer());
}