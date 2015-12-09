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
	cScenarioBallRLCacla::Init();
}

void cScenarioBallEval::ParseArgs(const cArgParser& parser)
{
	cScenarioBallRLCacla::ParseArgs(parser);
	parser.ParseInt("ball_eval_samples", mMaxSamples);
	parser.ParseString("output_file", mOutputFile);
}

void cScenarioBallEval::Reset()
{
	cScenarioBallRLCacla::Reset();
}

void cScenarioBallEval::Clear()
{
	cScenarioBallRLCacla::Clear();
}

void cScenarioBallEval::Update(double time_elapsed)
{
	cScenarioBallRLCacla::Update(time_elapsed);

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