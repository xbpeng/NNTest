#pragma once
#include "scenarios/DrawScenarioArmRL.h"

class cDrawScenarioArmRLPretrain : public cDrawScenarioArmRL
{
public:
	cDrawScenarioArmRLPretrain(cCamera& cam);
	virtual ~cDrawScenarioArmRLPretrain();

protected:
	virtual void BuildScene();
};