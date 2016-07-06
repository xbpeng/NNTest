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

void cArmNNTrackMuscularController::Update(double time_step)
{
	cArmNNTrackController::Update(time_step);
	UpdateMTUs(time_step);
}

void cArmNNTrackMuscularController::Reset()
{
	cArmNNTrackController::Reset();
	ResetMTUs();
}

int cArmNNTrackMuscularController::GetPoliActionSize() const
{
	return GetNumMTUs();
}

int cArmNNTrackMuscularController::GetNumMTUs() const
{
	return static_cast<int>(mMTUs.size());
}

const cMusculotendonUnit& cArmNNTrackMuscularController::GetMTU(int id) const
{
	return mMTUs[id];
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
				cMusculotendonUnit::tParams params;
				bool param_succ = cMusculotendonUnit::ParseParams(mtu_json, params);

				if (param_succ)
				{
					cMusculotendonUnit& curr_mtu = mMTUs[i];
					curr_mtu.Init(mChar, params);
				}
				else
				{
					succ = false;
					break;
				}
			}
		}

		if (!succ)
		{
			mMTUs.clear();
			printf("Failed to part MTUParams from %s\n", char_file.c_str());
			assert(false); // failed to parse MTU params
		}
	}
}

void cArmNNTrackMuscularController::UpdateMTUs(double time_step)
{
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		mMTUs[i].Update(time_step);
	}
}

void cArmNNTrackMuscularController::ResetMTUs()
{
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		mMTUs[i].Reset();
	}
}

void cArmNNTrackMuscularController::ApplyPoliAction(double time_step, const tAction& action)
{
	assert(action.mParams.size() == GetPoliActionSize());
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		double u = action.mParams[i];
		// hack
		u = 0.01;
		cMusculotendonUnit& mtu = mMTUs[i];
		mtu.SetExcitation(u);
	}
}