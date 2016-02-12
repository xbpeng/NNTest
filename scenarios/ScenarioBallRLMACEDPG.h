#pragma once

#include "scenarios/ScenarioBallRLDPG.h"

class cScenarioBallRLMACEDPG : public cScenarioBallRLDPG
{
public:
	cScenarioBallRLMACEDPG();
	virtual ~cScenarioBallRLMACEDPG();

	virtual void ParseArgs(const cArgParser& parser);
	virtual std::string GetName() const;
	
protected:
	virtual void InitTrainer();
	virtual void SetupController();
	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
};