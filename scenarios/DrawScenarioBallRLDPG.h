#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRLCacla.h"

class cDrawScenarioBallRLDPG : public cDrawScenarioBallRLCacla
{
public:
	cDrawScenarioBallRLDPG(cCamera& cam);
	virtual ~cDrawScenarioBallRLDPG();

protected:
	virtual void BuildScene();
};