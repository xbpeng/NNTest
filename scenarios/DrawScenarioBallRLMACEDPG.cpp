#include "DrawScenarioBallRLMACEDPG.h"
#include "scenarios/ScenarioBallRLMACEDPG.h"

cDrawScenarioBallRLMACEDPG::cDrawScenarioBallRLMACEDPG(cCamera& cam)
	: cDrawScenarioBallRLDPG(cam)
{
}

cDrawScenarioBallRLMACEDPG::~cDrawScenarioBallRLMACEDPG()
{
}

void cDrawScenarioBallRLMACEDPG::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLDPG>(new cScenarioBallRLMACEDPG());
}