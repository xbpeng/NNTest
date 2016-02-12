#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRLDPG.h"

class cDrawScenarioBallRLMACEDPG : public cDrawScenarioBallRLDPG
{
public:
	cDrawScenarioBallRLMACEDPG(cCamera& cam);
	virtual ~cDrawScenarioBallRLMACEDPG();

protected:
	virtual void BuildScene();
};