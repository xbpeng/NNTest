#pragma once

#include "scenarios/DrawScenarioArmTrain.h"

class cDrawScenarioArmTrainMACE : public cDrawScenarioArmTrain
{
public:
	cDrawScenarioArmTrainMACE(cCamera& cam);
	virtual ~cDrawScenarioArmTrainMACE();

protected:
	virtual void BuildScene();
};