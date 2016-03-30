#pragma once

#include "scenarios/ScenarioArmTrain.h"

class cScenarioArmTrainDPG : public cScenarioArmTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmTrainDPG();
	virtual ~cScenarioArmTrainDPG();

	virtual void Init();
	virtual std::string GetName() const;

protected:
	virtual void RecordFlagsEnd(tExpTuple& out_tuple) const;
	virtual void InitTrainer();
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;

	virtual void SetupCriticScale();
	
	virtual void PrintInfo() const;
};