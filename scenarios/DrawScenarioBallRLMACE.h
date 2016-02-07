#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLMACE.h"

class cDrawScenarioBallRLMACE : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLMACE(cCamera& cam);
	virtual ~cDrawScenarioBallRLMACE();

protected:

	virtual void BuildScene();
	virtual void UpdateTrace();
	virtual void DrawTrace() const;

	virtual const tVector& GetActionCol(int a_id) const;
};