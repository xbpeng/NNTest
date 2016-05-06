#pragma once
#include "ScenarioArm.h"
#include "anim/KinCharacter.h"

class cScenarioArmImitate : public cScenarioArm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArmImitate();
	virtual ~cScenarioArmImitate();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual std::string GetName() const;

protected:
	
	std::shared_ptr<cKinCharacter> mKinChar;

	virtual void BuildKinCharacter();
};