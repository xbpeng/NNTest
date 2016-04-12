#pragma once

#include "scenarios/DrawScenarioBallRLCacla.h"
#include "scenarios/ScenarioBallRLVarCacla.h"

class cDrawScenarioBallRLVarCacla : public cDrawScenarioBallRLCacla
{
public:
	cDrawScenarioBallRLVarCacla(cCamera& cam);
	virtual ~cDrawScenarioBallRLVarCacla();

protected:
	virtual void BuildScene();
};