#include "DrawScenarioBallRLMACE.h"

cDrawScenarioBallRLMACE::cDrawScenarioBallRLMACE(cCamera& cam)
	: cDrawScenarioBallRLACE(cam)
{
}

cDrawScenarioBallRLMACE::~cDrawScenarioBallRLMACE()
{
}

void cDrawScenarioBallRLMACE::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRLMACE>(new cScenarioBallRLMACE());
}