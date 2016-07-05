#include "ArmNNTrackMuscularController.h"
#include "sim/SimCharacter.h"

const std::string gMTUsKey = "MusculotendonUnits";

cArmNNTrackMuscularController::cArmNNTrackMuscularController()
{
}

cArmNNTrackMuscularController::~cArmNNTrackMuscularController()
{
}

void cArmNNTrackMuscularController::Init(cSimCharacter* character, const std::string& char_file)
{
	cArmNNTrackController::Init(character);
	BuildMTUs(char_file);
}

void cArmNNTrackMuscularController::Clear()
{
	cArmNNTrackController::Clear();
	mMTUs.clear();
}

void cArmNNTrackMuscularController::BuildMTUs(const std::string& char_file)
{
	std::ifstream f_stream(char_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gMTUsKey].isNull())
		{
			const Json::Value& mtus_arr = root[gMTUsKey];
			assert(mtus_arr.isArray());

			int num_mtus = mtus_arr.size();
			mMTUs.resize(num_mtus);

			for (int i = 0; i < num_mtus; ++i)
			{
				const Json::Value& mtu_json = mtus_arr.get(i, 0);

			}
		}
	}
}