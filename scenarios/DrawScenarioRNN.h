#pragma once
#include <memory>

#include "scenarios/DrawScenarioReg1D.h"
#include "ScenarioRNN.h"

class cDrawScenarioRNN : public cDrawScenarioReg1D
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioRNN(cCamera& cam);
	virtual ~cDrawScenarioRNN();

protected:

	virtual void BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene);
};