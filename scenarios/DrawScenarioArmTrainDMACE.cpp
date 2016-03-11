#include "DrawScenarioArmTrainDMACE.h"
#include "ScenarioArmTrainDMACE.h"

cDrawScenarioArmTrainDMACE::cDrawScenarioArmTrainDMACE(cCamera& cam)
	: cDrawScenarioArmTrainMACE(cam)
{
}

cDrawScenarioArmTrainDMACE::~cDrawScenarioArmTrainDMACE()
{
}

void cDrawScenarioArmTrainDMACE::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmTrainDMACE>(new cScenarioArmTrainDMACE());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}