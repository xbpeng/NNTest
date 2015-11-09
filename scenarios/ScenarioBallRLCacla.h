#pragma once

#include "scenarios/ScenarioBallRL.h"

class cScenarioBallRLCacla : public cScenarioBallRL
{
public:
	cScenarioBallRLCacla();
	virtual ~cScenarioBallRLCacla();

	virtual std::string GetName() const;
	
protected:
};