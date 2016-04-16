#pragma once

#include <string>
#include "util/MathUtil.h"
#include "scenarios/Scenario.h"
#include "learning/RecurrentNet.h"
#include "learning/RNNTrainer.h"

class cScenarioRNN : public cScenario
{
public:
	cScenarioRNN();
	virtual ~cScenarioRNN();

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

	std::vector<tVector, Eigen::aligned_allocator<tVector>> mPts;
	std::vector<tVector, Eigen::aligned_allocator<tVector>> mEvalPts;

	std::shared_ptr<cNeuralNetTrainer> mTrainer;

	virtual void InitTrainer();
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
	virtual void SetupScale();
	virtual void EvalNet();
	virtual void FindMinMaxX(double& out_min_x, double& out_max_x) const;
	virtual void BuildTuple(const tVector& pt0, const tVector& pt1, bool is_start, tExpTuple& out_tuple) const;
	
	virtual void GenPoints();
	virtual cRecurrentNet* GetNet() const;
};