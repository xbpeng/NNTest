#include "ArmNNTrackMuscularController.h"
#include "sim/SimCharacter.h"

#define ENABLE_MTU_STATE_FEATURES

const std::string gMTUsKey = "MusculotendonUnits";
const double gMinActivation = 0;
const double gMaxActivation = 1;


cArmNNTrackMuscularController::cArmNNTrackMuscularController()
{
	mExpNoise = 0.25;
}

cArmNNTrackMuscularController::~cArmNNTrackMuscularController()
{
}

void cArmNNTrackMuscularController::Init(cSimCharacter* character, const std::string& char_file)
{
	cArmNNTrackController::Init(character);
	BuildMTUs(char_file);
	InitPoliState();
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
	
	for (int i = 0; i < num_mtus; ++i)
	{
		double len = mMTUs[i].GetOptCELength();
		out_offset(mtu_state_offset + i) = -len;
		out_scale(mtu_state_offset + i) = 1 / len;
	}
#endif // ENABLE_MTU_STATE_FEATURES
}

void cArmNNTrackMuscularController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	const double activation_offset = 0;
	const double activation_scale = 1;
	int output_size = GetPoliActionSize();
	out_offset = activation_offset * Eigen::VectorXd::Ones(output_size);
	out_scale = activation_scale * Eigen::VectorXd::Ones(output_size);
}

int cArmNNTrackMuscularController::GetNumMTUs() const
{
	return static_cast<int>(mMTUs.size());
}

const cMusculotendonUnit& cArmNNTrackMuscularController::GetMTU(int id) const
{
	return mMTUs[id];
}

void cArmNNTrackMuscularController::HandlePoseReset()
{
	cArmNNTrackController::HandlePoseReset();
	ResetCEState();
}

void cArmNNTrackMuscularController::HandleVelReset()
{
	cArmNNTrackController::HandleVelReset();
	ResetCEState();
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
			printf("Failed to parse MTUParams from %s\n", char_file.c_str());
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

void cArmNNTrackMuscularController::ResetCEState()
{
	for (int i = 0; i < GetNumMTUs(); ++i)
	{
		mMTUs[i].ResetCELength();
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
		mtu_state(i) = mtu.GetCELength();
	}
#endif // ENABLE_MTU_STATE_FEATURES
}

void cArmNNTrackMuscularController::ApplyPoliAction(double time_step, const tAction& action)
{
	assert(action.mParams.size() == GetPoliActionSize());
	int num_mtus = GetNumMTUs();
	for (int i = 0; i < num_mtus; ++i)
	{
		double u = action.mParams[i];

		// hack
		u = 0;
		if (i == 0)
		{
			u = 1;
		}
		cMusculotendonUnit& mtu = mMTUs[i];
		mtu.SetExcitation(u);
	}
}

int cArmNNTrackMuscularController::GetMTUStateSize() const
{
	return GetNumMTUs();
}

void cArmNNTrackMuscularController::DecideAction()
{
	cArmNNTrackController::DecideAction();

	for (int i = 0; i < static_cast<int>(mPoliAction.mParams.size()); ++i)
	{
		mPoliAction.mParams[i] = cMathUtil::Clamp(mPoliAction.mParams[i], gMinActivation, gMaxActivation);
	}
}

void cArmNNTrackMuscularController::ApplyExpNoise(tAction& out_action) const
{
	// hack hack hack
	int action_size = GetPoliActionSize();
	Eigen::VectorXd bound_min = 0 * Eigen::VectorXd::Ones(action_size);
	Eigen::VectorXd bound_max = 1 * Eigen::VectorXd::Ones(action_size);
	out_action.mParams = out_action.mParams.cwiseMax(bound_min).cwiseMin(bound_max);
	
	cArmNNTrackController::ApplyExpNoise(out_action);
	/*
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	int action_size = static_cast<int>(out_action.mParams.size());
	Eigen::VectorXd exp_noise = Eigen::VectorXd::Zero(action_size);
	for (int i = 0; i < action_size; ++i)
	{
		double noise = cMathUtil::RandDoubleNorm(0, mExpNoise);
		double scale = noise_scale[i];
		noise *= scale;
		exp_noise[i] = noise;
	}

	for (int i = 0; i < action_size / 2; ++i)
	{
		int curr_idx = i * 2 + 1;
		int parent_idx = curr_idx - 1;
		double curr_noise = exp_noise[curr_idx];
		double parent_noise = exp_noise[parent_idx];
		curr_noise = -cMathUtil::Sign(parent_noise)  * std::abs(curr_noise);
		exp_noise[curr_idx] = curr_noise;
	}

	out_action.mParams += exp_noise;
	*/
}