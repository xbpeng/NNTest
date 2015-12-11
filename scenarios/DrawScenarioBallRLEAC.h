#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLEAC.h"

class cDrawScenarioBallRLEAC : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLEAC(cCamera& cam);
	virtual ~cDrawScenarioBallRLEAC();

protected:

	virtual void BuildScene();
	virtual void UpdateTrace();
	virtual void DrawTrace() const;

	virtual const tVector& GetActionCol(int a_id) const;
};