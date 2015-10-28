#pragma once

#include "scenarios/ScenarioReg1D.h"

class cScenarioReg1DTrainer : public cScenarioReg1D
{
public:
	cScenarioReg1DTrainer();
	virtual ~cScenarioReg1DTrainer();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual void AddPt(const tVector& pt);
	virtual void TrainNet();

	virtual std::string GetName() const;
	
protected:
	virtual void SetupTrainer();
};