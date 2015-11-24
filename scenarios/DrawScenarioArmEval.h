#pragma once
#include "scenarios/DrawScenarioArm.h"

class cDrawScenarioArmEval : public cDrawScenarioArm
{
public:
	cDrawScenarioArmEval(cCamera& cam);
	virtual ~cDrawScenarioArmEval();

	virtual void Keyboard(unsigned char key, int x, int y);

protected:

	virtual void BuildScene();
	virtual std::string BuildTextInfoStr() const;
	virtual void ToggleOutputData();
};