#include "SimArm.h"
#include <iostream>

const short gColFlag = cContactManager::gFlagNone; // tail is too small, so to prevent tunnelling disable all collisions

const short gBodyColFlags[cSimArm::eJointMax] =
{
	gColFlag,
	gColFlag,
	gColFlag,
	gColFlag,
	gColFlag
};

cSimArm::cSimArm()
{
}

cSimArm::~cSimArm()
{
}

short cSimArm::GetPartColGroup(int part_id) const
{
	return gBodyColFlags[part_id];
}

short cSimArm::GetPartColMask(int part_id) const
{
	return gBodyColFlags[part_id];
}

bool cSimArm::IsEndEffector(int idx) const
{
	return (idx == eJointLinkEnd);
}