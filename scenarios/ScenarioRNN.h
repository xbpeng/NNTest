#pragma once

#include <string>
#include "util/MathUtil.h"
#include "ScenarioReg1D.h"
#include "learning/RecurrentNet.h"
#include "learning/RNNTrainer.h"

class cScenarioRNN : public cScenarioReg1D
{
public:
	cScenarioRNN();
	virtual ~cScenarioRNN();

	virtual void Init();
	virtual void Reset();
	virtual void AddPt(const tVector& pt);

	virtual std::string GetName() const;
	
protected:

	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
	virtual void EvalNet();
	virtual void BuildTuple(const tVector& pt, bool is_start, tExpTuple& out_tuple) const;
	
	virtual void GenPoints();
	virtual cRecurrentNet* GetRNN() const;
};