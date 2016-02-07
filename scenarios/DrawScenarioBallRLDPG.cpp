#include "DrawScenarioBallRLDPG.h"
#include "scenarios/ScenarioBallRLDPG.h"

cDrawScenarioBallRLDPG::cDrawScenarioBallRLDPG(cCamera& cam)
	: cDrawScenarioBallRLCacla(cam)
{
}

cDrawScenarioBallRLDPG::~cDrawScenarioBallRLDPG()
{
}

void cDrawScenarioBallRLDPG::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLDPG>(new cScenarioBallRLDPG());
}