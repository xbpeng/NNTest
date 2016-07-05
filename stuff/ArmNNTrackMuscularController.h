#pragma once

#include "ArmNNTrackController.h"
#include "sim/MusculotendonUnit.h"

class cArmNNTrackMuscularController : public cArmNNTrackController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cArmNNTrackMuscularController();
	virtual ~cArmNNTrackMuscularController();

	virtual void Init(cSimCharacter* character, const std::string& char_file);
	virtual void Clear();

protected:

	std::vector<cMusculotendonUnit> mMTUs;
	virtual void BuildMTUs(const std::string& char_file);
};