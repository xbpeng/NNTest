#pragma once
#include "scenarios/DrawScenarioArmEval.h"

class cDrawScenarioArmImitateEval : public cDrawScenarioArmEval
{
public:
	cDrawScenarioArmImitateEval(cCamera& cam);
	virtual ~cDrawScenarioArmImitateEval();

protected:

	virtual void BuildScene();
	virtual void DrawCharacter();
	virtual void DrawKinChar();

	virtual void SetTarget(const tVector& target);
};