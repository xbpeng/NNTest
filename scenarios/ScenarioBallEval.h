#pragma once

#include "scenarios/ScenarioBallRLCacla.h"
#include "scenarios/ScenarioBallRLMACE.h"
#include "scenarios/ScenarioBallRLDPG.h"
#include "scenarios/ScenarioBallRLMACEDPG.h"
#include "scenarios/ScenarioBallRLVarCacla.h"

#define BALL_EVAL_BASE cScenarioBallRLCacla

class cScenarioBallEval : public BALL_EVAL_BASE
{
public:
	cScenarioBallEval();
	virtual ~cScenarioBallEval();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual bool EnableTraining() const;
	virtual bool IsDone() const;

	virtual std::string GetName() const;
	
protected:
	int mMaxSamples;
	std::string mOutputFile;
	bool mDoneOutput;

	virtual void OutputResult(const std::string& filename) const;
};