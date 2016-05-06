#include "ScenarioArmImitate.h"

cScenarioArmImitate::cScenarioArmImitate()
{
	mEnableAutoTarget = false;
	mTargetPos.setZero();
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
	mTrainerParams.mNumInitSamples = 200;

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
	cScenarioArmTrain::UpdateCharacter(time_step);
}

void cScenarioArmImitate::RandReset()
{
	cScenarioArmTrain::RandReset();
}

void cScenarioArmImitate::ResetKinChar()
{
	mKinChar->Reset();
}

void cScenarioArmImitate::RandResetKinChar()
{
	mKinChar->Reset();
	double dur = mKinChar->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	mKinChar->SetTime(rand_time);
	mKinChar->Update(0);
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
	Eigen::VectorXd kin_pose;
	Eigen::VectorXd kin_vel;
	mKinChar->BuildPose(kin_pose);
	mKinChar->BuildVel(kin_vel);
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
	double reward = pose_w * pose_reward + vel_w * vel_reward;

	if (CheckFail())
	{
		reward = 0;
	}

	return reward;
}

void cScenarioArmImitate::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 0.1;
	out_max = 0.2;
}