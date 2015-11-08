#include "ScenarioArmEval.h"
#include "stuff/SimArm.h"
#include "util/FileUtil.h"

const unsigned long int gRandSeed = 604;

cScenarioArmEval::cScenarioArmEval()
{
	mAvgErr = 0;
	mErrSampleCount = 0;
	mMaxSamples = std::numeric_limits<int>::max();
	mOutputFile = "";
}

cScenarioArmEval::~cScenarioArmEval()
{
}

void cScenarioArmEval::Init()
{
	cScenarioArm::Init();
	mAvgErr = 0;
	mErrSampleCount = 0;
	mRand.Seed(gRandSeed);
}

void cScenarioArmEval::ParseArgs(const cArgParser& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser.ParseInt("eval_max_samples", mMaxSamples);
	parser.ParseString("output_file", mOutputFile);
}

void cScenarioArmEval::Clear()
{
	cScenarioSimChar::Clear();
	mRenderTarget.reset();
	mAvgErr = 0;
	mErrSampleCount = 0;
}

void cScenarioArmEval::Update(double time_elapsed)
{
	cScenarioArm::Update(time_elapsed);
	UpdateTrackError();

	if (mErrSampleCount == mMaxSamples)
	{
		if (mOutputFile != "")
		{
			OutputResult(mOutputFile);
		}
	}
}

double cScenarioArmEval::GetAvgErr() const
{
	return mAvgErr;
}

int cScenarioArmEval::GetErrSampleCount() const
{
	return mErrSampleCount;
}

bool cScenarioArmEval::IsDone() const
{
	return mErrSampleCount >= mMaxSamples;
}

std::string cScenarioArmEval::GetName() const
{
	return "Arm Eval";
}

void cScenarioArmEval::UpdateTrackError()
{
	int end_id = cSimArm::eJointLinkEnd;
	tVector end_pos = mChar->CalcJointPos(end_id);

	tVector delta = mTargetPos - end_pos;
	delta[2] = 0;
	double err = delta.norm();
	
	mAvgErr = cMathUtil::AddAverage(mAvgErr, mErrSampleCount, err, 1);
	++mErrSampleCount;
}

void cScenarioArmEval::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 0.5;
	out_max = 1;
}

void cScenarioArmEval::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 1;
	out_max = 2;
}

double cScenarioArmEval::GetRandTargetMaxDist() const
{
	return cScenarioArm::GetRandTargetMaxDist();
}

void cScenarioArmEval::OutputResult(const std::string& filename) const
{
	std::string str = "";
	str += std::to_string(mAvgErr) + "\n";
	bool succ = cFileUtil::AppendText(str, filename);

	if (succ)
	{
		printf("Output result to %s\n", filename.c_str());
	}
	else
	{
		printf("Failed to output result to %s\n", filename.c_str());
	}
}