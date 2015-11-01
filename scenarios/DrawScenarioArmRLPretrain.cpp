#include "DrawScenarioArmRLPretrain.h"
#include "ScenarioArmRLPretrain.h"

cDrawScenarioArmRLPretrain::cDrawScenarioArmRLPretrain(cCamera& cam)
	: cDrawScenarioArmRL(cam)
{
}

cDrawScenarioArmRLPretrain::~cDrawScenarioArmRLPretrain()
{
}

void cDrawScenarioArmRLPretrain::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmRLPretrain>(new cScenarioArmRLPretrain());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}