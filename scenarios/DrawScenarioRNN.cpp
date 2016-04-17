#include "DrawScenarioRNN.h"

cDrawScenarioRNN::cDrawScenarioRNN(cCamera& cam)
	: cDrawScenarioReg1D(cam)
{
}

cDrawScenarioRNN::~cDrawScenarioRNN()
{
}

void cDrawScenarioRNN::BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene)
{
	out_scene = std::unique_ptr<cScenarioReg1D>(new cScenarioRNN());
}