#pragma once

#include "scenarios/ScenarioArmTrain.h"
#include "stuff/ArmControllerMACE.h"

class cScenarioArmTrainMACE : public cScenarioArmTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmTrainMACE();
	virtual ~cScenarioArmTrainMACE();

	virtual void Init();
	virtual std::string GetName() const;

protected:
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	virtual void RecordFlagsEnd(tExpTuple& out_tuple) const;
	
	virtual void InitTrainer();
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;

	virtual void SetupActorScale();
	virtual void SetupCriticScale();

	virtual void PrintInfo() const;

	virtual bool CheckExpCritic() const;
	virtual bool CheckExpActor() const;
	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;
	virtual std::shared_ptr<cArmControllerMACE> GetMACECtrl() const;
};