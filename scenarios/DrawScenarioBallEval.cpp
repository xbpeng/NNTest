#include "DrawScenarioBallEval.h"
#include "scenarios/ScenarioBallEval.h"

cDrawScenarioBallEval::cDrawScenarioBallEval(cCamera& cam)
	: cDrawScenarioBallRL(cam)
{
}

cDrawScenarioBallEval::~cDrawScenarioBallEval()
{
}

void cDrawScenarioBallEval::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallEval>(new cScenarioBallEval());
}