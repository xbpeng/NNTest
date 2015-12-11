#include "DrawScenarioBallRLQAC.h"

cDrawScenarioBallRLQAC::cDrawScenarioBallRLQAC(cCamera& cam)
	: cDrawScenarioBallRL(cam)
{
}

cDrawScenarioBallRLQAC::~cDrawScenarioBallRLQAC()
{
}

void cDrawScenarioBallRLQAC::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLQAC>(new cScenarioBallRLQAC());
}