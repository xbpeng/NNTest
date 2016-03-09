#include "DrawScenarioArmTrainMACE.h"
#include "ScenarioArmTrainMACE.h"

cDrawScenarioArmTrainMACE::cDrawScenarioArmTrainMACE(cCamera& cam)
	: cDrawScenarioArmTrain(cam)
{
}

cDrawScenarioArmTrainMACE::~cDrawScenarioArmTrainMACE()
{
}

void cDrawScenarioArmTrainMACE::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmTrainMACE>(new cScenarioArmTrainMACE());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}