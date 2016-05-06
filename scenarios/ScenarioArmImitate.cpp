#include "ScenarioArmImitate.h"

cScenarioArmImitate::cScenarioArmImitate()
{
	mEnableRandPose = false;
	mEnableAutoTarget = false;
	mTargetPos.setZero();
}

cScenarioArmImitate::~cScenarioArmImitate()
{
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
	bool succ = mKinChar->Init(mCharacterFile);

	if (!succ)
	{
		printf("Failed to load kin character from %s\n", mCharacterFile.c_str());
	}
}