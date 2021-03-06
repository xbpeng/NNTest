#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerMACE.h"
#include "learning/MACETrainer.h"

class cScenarioBallRLMACE : public cScenarioBallRL
{
public:
	cScenarioBallRLMACE();
	virtual ~cScenarioBallRLMACE();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual std::string GetName() const;
	
protected:

	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();

	virtual void RecordBegFlags(tExpTuple& out_tuple) const;
	virtual bool CheckExpCritic() const;
	virtual bool CheckExpActor() const;

	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual std::shared_ptr<cBallControllerMACE> GetACECtrl() const;
};