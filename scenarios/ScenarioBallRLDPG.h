#pragma once

#include "scenarios/ScenarioBallRLCacla.h"

class cScenarioBallRLDPG : public cScenarioBallRLCacla
{
public:
	cScenarioBallRLDPG();
	virtual ~cScenarioBallRLDPG();

	virtual void ParseArgs(const cArgParser& parser);
	virtual std::string GetName() const;
	
protected:
	virtual void InitTrainer();
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void NewCycleUpdate();
};