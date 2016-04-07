#pragma once
#include <memory>

#include "scenarios/DrawScenarioReg1D.h"

class cDrawScenarioRegVar1D : public cDrawScenarioReg1D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioRegVar1D(cCamera& cam);
	virtual ~cDrawScenarioRegVar1D();

protected:
	virtual void BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene);
	virtual void DrawNetEval() const;
};