#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLCacla.h"

class cDrawScenarioBallRLCacla : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLCacla(cCamera& cam);
	virtual ~cDrawScenarioBallRLCacla();

protected:
	
	virtual void BuildScene();
};