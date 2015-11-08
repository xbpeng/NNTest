#pragma once
#include "scenarios/DrawScenarioArm.h"

class cDrawScenarioArmEval : public cDrawScenarioArm
{
public:
	cDrawScenarioArmEval(cCamera& cam);
	virtual ~cDrawScenarioArmEval();

protected:

	virtual void BuildScene();
	virtual std::string BuildTextInfoStr() const;
};