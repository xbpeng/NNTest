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

	virtual void Init();
	virtual void Reset();

	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

protected:

	bool mNewSeq;

	virtual void BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene);
	virtual void AddPt(const tVector& pt);
};