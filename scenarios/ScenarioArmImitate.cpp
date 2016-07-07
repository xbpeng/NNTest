#include "ScenarioArmImitate.h"
#include "stuff/ArmNNTrackController.h"
#include "sim/CtTrackController.h"

#define ENABLE_RAND_KIN_RESET

cScenarioArmImitate::cScenarioArmImitate()
{
	mEnableAutoTarget = false;
	mTargetPos = tVector(1000, 1000, 0, 0);
	mMotionFile = "";
}

cScenarioArmImitate::~cScenarioArmImitate()
{
}

void cScenarioArmImitate::ParseArgs(const cArgParser& parser)
{
	cScenarioArmTrain::ParseArgs(parser);
	parser.ParseString("motion_file", mMotionFile);
}

void cScenarioArmImitate::Init()
{
	cScenarioArmTrain::Init();
	BuildKinCharacter();
	SyncCharacter();
}

void cScenarioArmImitate::Reset()
{
	ResetKinChar();
	cScenarioArmTrain::Reset();
	SyncCharacter();
}

void cScenarioArmImitate::Clear()
{
	cScenarioArmTrain::Clear();
	mKinChar->Clear();
}

const std::shared_ptr<cKinCharacter>& cScenarioArmImitate::GetKinChar() const
{
	return mKinChar;
}

void cScenarioArmImitate::ToggleTraining()
{
	cScenarioArmTrain::ToggleTraining();
	mEnableRandPose = EnableTraining();
}

std::string cScenarioArmImitate::GetName() const
{
	return "Arm Imitate";
}

void cScenarioArmImitate::BuildKinCharacter()
{
	mKinChar = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	mKinChar->EnableVelUpdate(true);
	bool succ = mKinChar->Init(mCharacterFile, mMotionFile);

	if (!succ)
	{
		printf("Failed to load kin character from %s\n", mCharacterFile.c_str());
	}
}

void cScenarioArmImitate::InitTrainer()
{
	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mNetFile = mCriticNetFile;
	mTrainerParams.mPlaybackMemSize = 25000;
	mTrainerParams.mPoolSize = 1;
	mTrainerParams.mInitInputOffsetScale = false;
	mTrainerParams.mNumInitSamples = 20000;

	mTrainer->Init(mTrainerParams);
	SetupScale();

	if (mCriticModelFile != "")
	{
		mTrainer->LoadCriticModel(mCriticModelFile);
	}

	if (mActorModelFile != "")
	{
		mTrainer->LoadActorModel(mActorModelFile);
	}

	SetupActionBounds();
}

void cScenarioArmImitate::UpdateCharacter(double time_step)
{
	mKinChar->Update(time_step);
	UpdateArmTrackController();
	cScenarioArmTrain::UpdateCharacter(time_step);
}

void cScenarioArmImitate::RandReset()
{
	mKinChar->Reset();
	cScenarioArmTrain::RandReset();
}

void cScenarioArmImitate::ResetKinChar()
{
	mKinChar->Reset();
}

void cScenarioArmImitate::RandResetKinChar()
{
	mKinChar->Reset();

#if defined(ENABLE_RAND_KIN_RESET)
	double dur = mKinChar->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	mKinChar->SetTime(rand_time);
	mKinChar->Update(0);
#endif
}

void cScenarioArmImitate::ApplyRandPose()
{
	RandResetKinChar();
	SyncCharacter();
	ResetPoseCounter();
	mValidSample = false;
}

void cScenarioArmImitate::SyncCharacter()
{
	const Eigen::VectorXd& kin_pose = mKinChar->GetPose();
	const Eigen::VectorXd& kin_vel = mKinChar->GetVel();
	mChar->SetPose(kin_pose);
	mChar->SetVel(kin_vel);
}

double cScenarioArmImitate::CalcReward() const
{
	const double pose_w = 0.9;
	const double vel_w = 0.1;
	double pos_err = 0;
	double vel_err = 0;
	int num_joints = mChar->GetNumJoints();

	/*
	for (int j = 0; j < num_joints; ++j)
	{
	tVector joint_pos = mChar->CalcJointPos(j);
	tVector kin_pos = mKinChar->CalcJointPos(j);
	double curr_pos_err = (kin_pos - joint_pos).squaredNorm();
	pos_err += curr_pos_err;

	tVector joint_vel = mChar->CalcJointVel(j);
	tVector kin_vel = mKinChar->CalcJointVel(j);
	double curr_vel_err = (kin_vel - joint_vel).squaredNorm();
	vel_err += curr_vel_err;
	}

	double pose_reward = exp(-5 * pos_err / num_joints);
	double vel_reward = exp(-0.1 * vel_err / num_joints);
	*/

	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& vel = mChar->GetVel();
	const Eigen::VectorXd& kin_pose = mKinChar->GetPose();
	const Eigen::VectorXd& kin_vel = mKinChar->GetVel();

	Eigen::VectorXd pose_diff = kin_pose - pose;
	Eigen::VectorXd vel_diff = kin_vel - vel;

	for (int j = 0; j < num_joints; ++j)
	{
		if (j != 0)
		{
			int offset = mChar->GetParamOffset(j);
			double curr_pose_err = std::abs(pose_diff[offset]);
			double curr_vel_err = std::abs(vel_diff[offset]);
			curr_pose_err = std::min(2 * M_PI - curr_pose_err, curr_pose_err);

			pos_err += curr_pose_err * curr_pose_err;
			vel_err += curr_vel_err * curr_vel_err;
		}
	}

	double pose_reward = exp(-0.5 * pos_err / num_joints);
	double vel_reward = exp(-0.01 * vel_err / num_joints);

	double reward = pose_w * pose_reward + vel_w * vel_reward;

	if (CheckFail())
	{
		reward = 0;
	}

	return reward;
}

void cScenarioArmImitate::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	//out_min = 0.1;
	//out_max = 0.5;
	out_min = 6;
	out_max = 8;
}

void cScenarioArmImitate::UpdateArmTrackController()
{
	auto track_ctrl = std::dynamic_pointer_cast<cArmNNTrackController>(mChar->GetController());
	if (track_ctrl != nullptr)
	{
		Eigen::VectorXd pose;
		Eigen::VectorXd vel;

		double kin_time = mKinChar->GetTime();
		double ctrl_period = track_ctrl->GetUpdatePeriod();
		double next_time = kin_time;// +ctrl_period;
		mKinChar->CalcPose(next_time, pose);
		mKinChar->CalcVel(next_time, vel);
		track_ctrl->SetTargetPoseVel(pose, vel);
	}
}