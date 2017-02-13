#include "ScenarioArmImitateEval.h"

#define ENABLE_RAND_KIN_RESET

cScenarioArmImitateEval::cScenarioArmImitateEval()
{
	mEnableAutoTarget = false;
	mTargetPos = tVector(1000, 1000, 0, 0);
	mMotionFile = "";
}

cScenarioArmImitateEval::~cScenarioArmImitateEval()
{
}

void cScenarioArmImitateEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioArmEval::ParseArgs(parser);
	parser->ParseString("motion_file", mMotionFile);
}

void cScenarioArmImitateEval::Init()
{
	cScenarioArmEval::Init();
	BuildKinCharacter();
	SyncCharacter();
}


void cScenarioArmImitateEval::Reset()
{
	ResetKinChar();
	cScenarioArmEval::Reset();
	SyncCharacter();
}

void cScenarioArmImitateEval::Clear()
{
	cScenarioArmEval::Clear();
	mKinChar->Clear();
}

const std::shared_ptr<cKinCharacter>& cScenarioArmImitateEval::GetKinChar() const
{
	return mKinChar;
}

std::string cScenarioArmImitateEval::GetName() const
{
	return "Arm Imitate Eval";
}

void cScenarioArmImitateEval::BuildKinCharacter()
{
	mKinChar = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	mKinChar->EnableVelUpdate(true);
	bool succ = mKinChar->Init(mCharacterFile, mMotionFile);

	if (!succ)
	{
		printf("Failed to load kin character from %s\n", mCharacterFile.c_str());
	}
}

void cScenarioArmImitateEval::SyncCharacter()
{
	const Eigen::VectorXd& kin_pose = mKinChar->GetPose();
	const Eigen::VectorXd& kin_vel = mKinChar->GetVel();
	mChar->SetPose(kin_pose);
	mChar->SetVel(kin_vel);
}

void cScenarioArmImitateEval::ResetKinChar()
{
	mKinChar->Reset();
}

void cScenarioArmImitateEval::UpdateCharacter(double time_step)
{
	mKinChar->Update(time_step);
	UpdateArmTrackController();
	cScenarioArmEval::UpdateCharacter(time_step);
}

void cScenarioArmImitateEval::UpdateArmTrackController()
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

double cScenarioArmImitateEval::CalcError() const
{
	const Eigen::VectorXd& pose = mChar->GetPose();
	const Eigen::VectorXd& kin_pose = mKinChar->GetPose();

	Eigen::VectorXd pose_diff = kin_pose - pose;
	int num_joints = mChar->GetNumJoints();
	int root_id = mChar->GetRootID();
	const auto& joint_desc = mChar->GetJointMat();
	double pose_err = 0;

	tVector root_diff = cKinTree::GetRootPos(joint_desc, pose_diff);
	root_diff[0] = 0;
	cKinTree::SetRootPos(joint_desc, root_diff, pose_diff);

	double total_w = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		int offset = mChar->GetParamOffset(j);
		int size = mChar->GetParamSize(j);
		double w = cKinTree::GetJointDiffWeight(joint_desc, j);

		double curr_pose_diff = pose_diff.segment(offset, size).squaredNorm();
		pose_err += w * curr_pose_diff;
		total_w += w;
	}
	pose_err /= total_w;
	return pose_err;
}

void cScenarioArmImitateEval::RandReset()
{
	ResetKinChar();
	cScenarioArmEval::RandReset();

	if (mEnableRandPose)
	{
		RandResetKinChar();
	}
}

void cScenarioArmImitateEval::ApplyRandPose()
{
	RandResetKinChar();
	SyncCharacter();
	ResetPoseCounter();
}

void cScenarioArmImitateEval::RandResetKinChar()
{
	mKinChar->Reset();

#if defined(ENABLE_RAND_KIN_RESET)
	double dur = mKinChar->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	mKinChar->SetTime(rand_time);
	mKinChar->Update(0);
#endif
}

void cScenarioArmImitateEval::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 10;
	out_max = 20;
}