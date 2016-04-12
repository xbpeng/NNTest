#pragma once

#include "scenarios/ScenarioBallRLCacla.h"
#include "stuff/BallControllerVarCacla.h"

class cScenarioBallRLVarCacla : public cScenarioBallRLCacla
{
public:
	cScenarioBallRLVarCacla();
	virtual ~cScenarioBallRLVarCacla();

	virtual std::string GetName() const;
	
protected:

	virtual void BuildController(std::shared_ptr<cBallController>& out_ctrl);
	virtual void InitTrainer();
};