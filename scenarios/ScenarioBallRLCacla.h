#pragma once

#include "scenarios/ScenarioBallRL.h"
#include "stuff/BallControllerCont.h"
#include "learning/CaclaTrainer.h"

class cScenarioBallRLCacla : public cScenarioBallRL
{
public:
	cScenarioBallRLCacla();
	virtual ~cScenarioBallRLCacla();

	virtual void ParseArgs(const cArgParser& parser);

	virtual std::string GetName() const;
	
protected:
	std::string mActorSolverFile;
	std::string mActorNetFile;

	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
};