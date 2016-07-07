#include "ArmNNTrackMuscularController.h"
#include "sim/SimCharacter.h"

//#define ENABLE_MTU_STATE_FEATURES

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

int cArmNNTrackMuscularController::GetPoliStateSize() const
{
	int state_size = cArmNNTrackController::GetPoliStateSize();

#if defined(ENABLE_MTU_STATE_FEATURES)
	state_size += GetMTUStateSize();
#endif // ENABLE_MTU_STATE_FEATURES

	return state_size;
}

int cArmNNTrackMuscularController::GetPoliActionSize() const
{
	return GetNumMTUs();
}

void cArmNNTrackMuscularController::BuildNNInputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	cArmNNTrackController::BuildNNInputOffsetScale(out_offset, out_scale);

#if defined(ENABLE_MTU_STATE_FEATURES)
	int mtu_state_offset = cArmNNTrackController::GetPoliStateSize();
	int mtu_state_size = GetMTUStateSize();
	int num_mtus = GetNumMTUs();
	
	const double activation_offset = -0.5;
	const double activation_scale = 2;
	for (int i = 0; i < num_mtus; ++i)
	{
		out_offset(mtu_state_offset + i) = activation_offset;
		out_scale(mtu_state_offset + i) = activation_scale;

		double len = mMTUs[i].GetOptCELength();
		out_offset(mtu_state_offset + num_mtus + i) = -0.5 * len;
		out_scale(mtu_state_offset + num_mtus + i) = 2 / len;
	}
#endif // ENABLE_MTU_STATE_FEATURES
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

void cArmNNTrackMuscularController::UpdatePoliState()
{
	cArmNNTrackController::UpdatePoliState();

#if defined(ENABLE_MTU_STATE_FEATURES)
	int mtu_state_offset = cArmNNTrackController::GetPoliStateSize();
	int mtu_state_size = GetMTUStateSize();
	auto mtu_state = mPoliState.segment(mtu_state_offset, mtu_state_size);
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		const auto& mtu = mMTUs[i];
		mtu_state(i) = mtu.GetActivation();
		mtu_state(num_mtus + i) = mtu.GetCELength();
	}
#endif // ENABLE_MTU_STATE_FEATURES
}

void cArmNNTrackMuscularController::ApplyPoliAction(double time_step, const tAction& action)
{
	static double time = 0;
	time += time_step;

	assert(action.mParams.size() == GetPoliActionSize());
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		double u = action.mParams[i];
		cMusculotendonUnit& mtu = mMTUs[i];
		mtu.SetExcitation(u);
	}
}

int cArmNNTrackMuscularController::GetMTUStateSize() const
{
	return 2 * GetNumMTUs();
}