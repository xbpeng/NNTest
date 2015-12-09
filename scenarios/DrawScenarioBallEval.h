#pragma once
#include "scenarios/DrawScenarioBallRL.h"

class cDrawScenarioBallEval : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallEval(cCamera& cam);
	virtual ~cDrawScenarioBallEval();

protected:
	
	virtual void BuildScene();
};