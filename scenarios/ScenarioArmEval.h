#pragma once

#include "ScenarioArm.h"

class cScenarioArmEval : public cScenarioArm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmEval();
	virtual ~cScenarioArmEval();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Clear();

	virtual void Update(double time_elapsed);
	virtual double GetAvgErr() const;
	virtual int GetErrSampleCount() const;

	virtual bool IsDone() const;

	virtual std::string GetName() const;

protected:
	double mAvgErr;
	int mErrSampleCount;
	int mMaxSamples;

	std::string mOutputFile;

	virtual void UpdateTrackError();
	virtual void GetRandTargetMinMaxTime(double& out_min, double& out_max) const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
	virtual double GetRandTargetMaxDist() const;

	virtual void OutputResult(const std::string& filename) const;
};