#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRLMACE.h"

class cDrawScenarioBallRLMACEDPG : public cDrawScenarioBallRLMACE
{
public:
	cDrawScenarioBallRLMACEDPG(cCamera& cam);
	virtual ~cDrawScenarioBallRLMACEDPG();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

protected:
	std::string mCriticOutputNetFile;

	virtual void BuildScene();
	virtual void SaveNet() const;
};