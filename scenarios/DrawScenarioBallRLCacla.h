#pragma once
#include <memory>

#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/ScenarioBallRLCacla.h"

class cDrawScenarioBallRLCacla : public cDrawScenarioBallRL
{
public:
	cDrawScenarioBallRLCacla(cCamera& cam);
	virtual ~cDrawScenarioBallRLCacla();

	virtual void ParseArgs(const cArgParser& parser);

protected:
	std::string mCriticOutputNetFile;

	virtual void BuildScene();
	virtual void SaveNet() const;
};