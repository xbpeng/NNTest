#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLQAC.h"

class cDrawScenarioBallRLQAC : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLQAC(cCamera& cam);
	virtual ~cDrawScenarioBallRLQAC();

protected:

	virtual void BuildScene();
};