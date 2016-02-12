#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRLMACE.h"

class cDrawScenarioBallRLMACEDPG : public cDrawScenarioBallRLMACE
{
public:
	cDrawScenarioBallRLMACEDPG(cCamera& cam);
	virtual ~cDrawScenarioBallRLMACEDPG();

protected:
	virtual void BuildScene();
};