#pragma once

#include "scenarios/DrawScenarioArmTrain.h"

class cDrawScenarioArmTrainDPG : public cDrawScenarioArmTrain
{
public:
	cDrawScenarioArmTrainDPG(cCamera& cam);
	virtual ~cDrawScenarioArmTrainDPG();

protected:
	virtual void BuildScene();
};