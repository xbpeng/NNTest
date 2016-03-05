#include "DrawScenarioArmTrainDPG.h"
#include "ScenarioArmTrainDPG.h"

cDrawScenarioArmTrainDPG::cDrawScenarioArmTrainDPG(cCamera& cam)
	: cDrawScenarioArmTrain(cam)
{
}

cDrawScenarioArmTrainDPG::~cDrawScenarioArmTrainDPG()
{
}

void cDrawScenarioArmTrainDPG::BuildScene()
{
	mScene = std::shared_ptr<cScenarioArmTrainDPG>(new cScenarioArmTrainDPG());
	mSimScene = std::static_pointer_cast<cScenarioSimChar>(mScene); // arg hack
}