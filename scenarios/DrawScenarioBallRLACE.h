#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLACE.h"

class cDrawScenarioBallRLACE : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLACE(cCamera& cam);
	virtual ~cDrawScenarioBallRLACE();

protected:

	virtual void BuildScene();
	virtual void UpdateTrace();
	virtual void DrawTrace() const;

	virtual const tVector& GetActionCol(int a_id) const;
};