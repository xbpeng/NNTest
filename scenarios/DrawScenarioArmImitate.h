#pragma once

#include "DrawScenarioArm.h"

class cDrawScenarioArmImitate : public cDrawScenarioArm
{
public:
	cDrawScenarioArmImitate(cCamera& cam);
	virtual ~cDrawScenarioArmImitate();

protected:
	
	virtual void BuildScene();
	virtual void SetTarget(const tVector& target);

	virtual void DrawCharacter();
	virtual void DrawKinChar();
};