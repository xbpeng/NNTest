#include "DrawScenarioBallRLMACEDPG.h"
#include "scenarios/ScenarioBallRLMACEDPG.h"
#include "stuff/BallControllerMACEDPG.h"
#include "render/DrawUtil.h"

cDrawScenarioBallRLMACEDPG::cDrawScenarioBallRLMACEDPG(cCamera& cam)
	: cDrawScenarioBallRLMACE(cam)
{
}

cDrawScenarioBallRLMACEDPG::~cDrawScenarioBallRLMACEDPG()
{
}

void cDrawScenarioBallRLMACEDPG::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLDPG>(new cScenarioBallRLMACEDPG());
}