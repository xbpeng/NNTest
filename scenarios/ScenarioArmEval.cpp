#include "ScenarioArmEval.h"
#include "stuff/SimArm.h"
#include "util/FileUtil.h"

const unsigned long int gRandSeed = 604;
const std::string gErrFile = "output/arm_eval_err.txt";
const std::string gActionFile = "output/arm_eval_action.txt";

cScenarioArmEval::cScenarioArmEval()
{
	mAvgErr = 0;
	mErrSampleCount = 0;
	mMaxSamples = std::numeric_limits<int>::max();
	mOutputFile = "";

	mOutputData = false;
	mErrFile = nullptr;
	mActionFile = nullptr;
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
	ResetTargetCounter();
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

	if (mOutputData)
	{
		OutputData();
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
	auto arm_ctrl = GetArmController();
	int end_id = arm_ctrl->GetEndEffectorID();
	tVector end_pos = mChar->CalcJointPos(end_id);

	tVector delta = mTargetPos - end_pos;
	delta[2] = 0;
	double err = delta.norm();
	
	mAvgErr = cMathUtil::AddAverage(mAvgErr, mErrSampleCount, err, 1);
	++mErrSampleCount;
}

void cScenarioArmEval::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 1;
	out_max = 2;
}

void cScenarioArmEval::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 2;
	out_max = 4;
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


bool cScenarioArmEval::EnabledOutputData() const
{
	return mOutputData;
}

void cScenarioArmEval::EnableOutputData(bool enable)
{
	mOutputData = enable;

	if (mOutputData)
	{
		mErrFile = cFileUtil::OpenFile(gErrFile, "w");
		mActionFile = cFileUtil::OpenFile(gActionFile, "w");
	}
	else
	{
		cFileUtil::CloseFile(mErrFile);
		cFileUtil::CloseFile(mActionFile);
	}
}

void cScenarioArmEval::OutputData() const
{
	const auto& ctrl = mChar->GetController();
	if (ctrl != nullptr)
	{
		auto arm_ctrl = std::static_pointer_cast<cArmController>(ctrl);

		int end_id = arm_ctrl->GetEndEffectorID();
		tVector end_pos = mChar->CalcJointPos(end_id);
		
		tVector err = mTargetPos - end_pos;
		
		fprintf(mErrFile, "%.5f\t%.5f\n", err[0], err[1]);

		Eigen::VectorXd action;
		arm_ctrl->RecordPoliAction(action);
		
		for (int i = 0; i < action.size(); ++i)
		{
			fprintf(mActionFile, "%.5f\t", action[i]);
		}
		
		fprintf(mActionFile, "\n");
	}
}