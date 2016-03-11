#pragma once

#include "scenarios/DrawScenarioArmTrainMACE.h"

class cDrawScenarioArmTrainDMACE : public cDrawScenarioArmTrainMACE
{
public:
	cDrawScenarioArmTrainDMACE(cCamera& cam);
	virtual ~cDrawScenarioArmTrainDMACE();

protected:
	virtual void BuildScene();
};