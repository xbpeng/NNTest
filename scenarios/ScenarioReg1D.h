#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/Scenario.h"
#include "learning/NeuralNet.h"

class cScenarioReg1D : public cScenario
{
public:
	cScenarioReg1D();
	virtual ~cScenarioReg1D();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual int GetNumPts() const;
	virtual const tVector& GetPt(int i) const;
	virtual void AddPt(const tVector& pt);
	virtual const std::vector<tVector, Eigen::aligned_allocator<tVector>>& GetEvalPts() const;

	virtual void TrainNet();

	virtual std::string GetName() const;
	
protected:
	std::string mSolverFile;
	std::string mNetFile;
	int mPassesPerStep;

	int mNumEvalPts;
	std::vector<tVector, Eigen::aligned_allocator<tVector>> mPts;
	std::vector<tVector, Eigen::aligned_allocator<tVector>> mEvalPts;

	cNeuralNet mNet;

	virtual void SetupNet();
	virtual void BuildProb(cNeuralNet::tProblem& out_prob) const;
	virtual void EvalNet();
	virtual void FindMinMaxX(double& out_min_x, double& out_max_x) const;
};