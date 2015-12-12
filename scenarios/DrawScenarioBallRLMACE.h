#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRLACE.h"
#include "scenarios/ScenarioBallRLMACE.h"

class cDrawScenarioBallRLMACE : public cDrawScenarioBallRLACE
{
public:
	cDrawScenarioBallRLMACE(cCamera& cam);
	virtual ~cDrawScenarioBallRLMACE();

protected:

	virtual void BuildScene();
};