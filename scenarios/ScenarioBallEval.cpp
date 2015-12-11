#include "ScenarioBallEval.h"
#include "util/FileUtil.h"

cScenarioBallEval::cScenarioBallEval()
{
	mMaxSamples = 500;
	mOutputFile = "";
	mDoneOutput = false;
}

cScenarioBallEval::~cScenarioBallEval()
{
}

void cScenarioBallEval::Init()
{
	BALL_EVAL_BASE::Init();
}

void cScenarioBallEval::ParseArgs(const cArgParser& parser)
{
	BALL_EVAL_BASE::ParseArgs(parser);
	parser.ParseInt("ball_eval_samples", mMaxSamples);
	parser.ParseString("output_file", mOutputFile);
}

void cScenarioBallEval::Reset()
{
	BALL_EVAL_BASE::Reset();
}

void cScenarioBallEval::Clear()
{
	BALL_EVAL_BASE::Clear();
}

void cScenarioBallEval::Update(double time_elapsed)
{
	BALL_EVAL_BASE::Update(time_elapsed);

	if (IsDone())
	{
		if (mOutputFile != "" && !mDoneOutput)
		{
			OutputResult(mOutputFile);
			mDoneOutput = true;
		}
	}
}

bool cScenarioBallEval::EnableTraining() const
{
	return false;
}

bool cScenarioBallEval::IsDone() const
{
	int count = mGround.GetBoxCount();
	return count >= mMaxSamples;
}

std::string cScenarioBallEval::GetName() const
{
	return "Ball Eval";
}

void cScenarioBallEval::OutputResult(const std::string& filename) const
{
	double succ_rate = GetSuccRate();

	std::string str = "";
	str += std::to_string(succ_rate) + "\n";
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