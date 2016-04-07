#pragma once

#include "scenarios/ScenarioReg1D.h"

class cScenarioRegVar1D : public cScenarioReg1D
{
public:
	cScenarioRegVar1D();
	virtual ~cScenarioRegVar1D();

	virtual void Init();
	virtual void Reset();

	virtual std::string GetName() const;
	
protected:
	cRand mRand;

	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
	virtual void GenPoints();
	virtual tVector BuildPt(const Eigen::VectorXd& x, const Eigen::VectorXd& y) const;
};