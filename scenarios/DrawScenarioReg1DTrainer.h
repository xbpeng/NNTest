#pragma once
#include <memory>

#include "scenarios/DrawScenarioReg1D.h"

class cDrawScenarioReg1DTrainer : public cDrawScenarioReg1D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioReg1DTrainer(cCamera& cam);
	virtual ~cDrawScenarioReg1DTrainer();

protected:
	virtual void BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene);
};