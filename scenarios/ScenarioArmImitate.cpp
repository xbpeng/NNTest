#include "ScenarioArmImitate.h"

cScenarioArmImitate::cScenarioArmImitate()
{
	mEnableRandPose = false;
	mEnableAutoTarget = false;
	mTargetPos.setZero();
	mMotionFile = "";
}

cScenarioArmImitate::~cScenarioArmImitate()
{
}

void cScenarioArmImitate::ParseArgs(const cArgParser& parser)
{
	cScenarioArm::ParseArgs(parser);
	parser.ParseString("motion_file", mMotionFile);
}

void cScenarioArmImitate::Init()
{
	cScenarioArm::Init();
	BuildKinCharacter();
}

void cScenarioArmImitate::Reset()
{
	cScenarioArm::Init();
	mKinChar->Reset();
}

void cScenarioArmImitate::Clear()
{
	cScenarioArm::Clear();
	mKinChar->Clear();
}

const std::shared_ptr<cKinCharacter>& cScenarioArmImitate::GetKinChar() const
{
	return mKinChar;
}

std::string cScenarioArmImitate::GetName() const
{
	return "Arm Imitate";
}

void cScenarioArmImitate::BuildKinCharacter()
{
	mKinChar = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	bool succ = mKinChar->Init(mCharacterFile, mMotionFile);

	if (!succ)
	{
		printf("Failed to load kin character from %s\n", mCharacterFile.c_str());
	}
}

void cScenarioArmImitate::UpdateCharacter(double time_step)
{
	mKinChar->Update(time_step);
	cScenarioArm::UpdateCharacter(time_step);
}