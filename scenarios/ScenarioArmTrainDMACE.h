#pragma once

#include "scenarios/ScenarioArmTrainMACE.h"

class cScenarioArmTrainDMACE : public cScenarioArmTrainMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmTrainDMACE();
	virtual ~cScenarioArmTrainDMACE();

	virtual std::string GetName() const;

protected:
	
	virtual void InitTrainer();
	virtual void InitLearner();
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	
	virtual void SetupActorScale();
	virtual void SetupCriticScale();
	virtual double CalcExpTemp() const;

	virtual void Train();
};