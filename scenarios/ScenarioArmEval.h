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

	virtual bool EnabledOutputData() const;
	virtual void EnableOutputData(bool enable);

	virtual std::string GetName() const;

protected:
	double mAvgErr;
	int mErrSampleCount;
	int mMaxSamples;

	std::string mOutputFile;

	FILE* mErrFile;
	FILE* mActionFile;
	bool mOutputData;

	bool mRecordActions;
	std::string mActionOutputFile;
	bool mRecordActionIDState;
	std::string mActionIDStateOutputFile;

	virtual void UpdateCharacter(double time_step);

	virtual void UpdateTrackError();
	virtual double CalcError() const;

	virtual void GetRandTargetMinMaxTime(double& out_min, double& out_max) const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
	virtual double GetRandTargetMaxDist() const;
	virtual std::shared_ptr<cArmNNController> GetController() const;

	virtual void OutputResult(const std::string& filename) const;
	virtual void OutputData() const;

	virtual void InitActionRecord(const std::string& out_file) const;
	virtual bool EnableRecordActions() const;
	virtual void RecordAction(const std::string& out_file);
	
	virtual void InitActionIDStateRecord(const std::string& out_file) const;
	virtual bool EnableRecordActionIDState() const;
	virtual void RecordActionIDState(const std::string& out_file);
	virtual void PrintInfo() const;
};