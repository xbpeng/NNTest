#pragma once

#include "scenarios/ScenarioBallRLCacla.h"

class cScenarioBallEval : public cScenarioBallRLCacla
{
public:
	cScenarioBallEval();
	virtual ~cScenarioBallEval();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
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