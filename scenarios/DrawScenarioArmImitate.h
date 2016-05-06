#pragma once

#include "DrawScenarioArmTrain.h"

class cDrawScenarioArmImitate : public cDrawScenarioArmTrain
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