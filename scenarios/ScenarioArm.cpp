#include "ScenarioArm.h"
#include "stuff/SimArm.h"
#include "stuff/ArmNNPixelController.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const double gCamSize = 4;
const int gRTSize = 128;
const double gTargetRadius = 0.15;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const double gTorqueLim = 300;
const double gCtrlUpdatePeriod = 1 / 120.0;

const double gLinearDamping = 0;
const double gAngularDamping = 0;

cScenarioArm::cScenarioArm()
{
	mEnableTraining = true;
	mEnableAutoTarget = true;
	mEnableRandPose = true;
	mPretrain = false;
	mSimStepsPerUpdate = 5;
	mTargetPos = tVector(1, 0, 0, 0);
	ResetTargetCounter();
	ResetPoseCounter();

	mCtrlType = eCtrlQP;
	mGravity = tVector(0, 0, 0, 0);
}

cScenarioArm::~cScenarioArm()
{
}

void cScenarioArm::Init()
{
	cScenarioSimChar::Init();
	BuildCoach();

	InitTrainer();
	InitTupleBuffer();
	InitViewBuffer();
	Reset();

	InitRenderResources();
	InitCam();
	ResetTargetCounter();
	ResetPoseCounter();
}

void cScenarioArm::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseStringArray("model_file", mModelFiles);
	parser.ParseString("scale_file", mScaleFile);
	parser.ParseBool("arm_pretrain", mPretrain);

	ParseCtrlType(parser, mCtrlType);
}

void cScenarioArm::Reset()
{
	//cScenarioSimChar::Reset();
	//mCoach->Reset();
	ResetTargetCounter();
	RandReset();
}

void cScenarioArm::Clear()
{
	cScenarioSimChar::Clear();
	mRenderTarget.reset();
	mCoach->Clear();

	EnableOutputData(false);
}

void cScenarioArm::Update(double time_elapsed)
{
	UpdateTargetCounter(time_elapsed);
	UpdatePoseCounter(time_elapsed);
	SetCtrlTargetPos(mTargetPos);
	cScenarioSimChar::Update(time_elapsed);

	bool exploded = HasExploded();
	if (exploded)
	{
		RandReset();
	}
}

void cScenarioArm::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;
}

bool cScenarioArm::EnableTraining() const
{
	return mEnableTraining;
}

bool cScenarioArm::EnabledAutoTarget() const
{
	return mEnableAutoTarget;
}

void cScenarioArm::EnableAutoTarget(bool enable)
{
	mEnableAutoTarget = enable;
}

bool cScenarioArm::EnabledRandPose() const
{
	return mEnableRandPose;
}

void cScenarioArm::EnableRandPose(bool enable)
{
	mEnableRandPose = enable;
}

void cScenarioArm::SetTargetPos(const tVector& target)
{
	mTargetPos = target;
	double min_dist = -0.5 * gCamSize + gTargetRadius;
	double max_dist = 0.5 * gCamSize - gTargetRadius;
	mTargetPos[0] = cMathUtil::Clamp(mTargetPos[0], min_dist, max_dist);
	mTargetPos[1] = cMathUtil::Clamp(mTargetPos[1], min_dist, max_dist);
	ResetTargetCounter();
}

const tVector& cScenarioArm::GetTargetPos() const
{
	return mTargetPos;
}

void cScenarioArm::DrawCharacter() const
{
	DrawArm(mChar, gFillTint, gLineColor);
}

void cScenarioArm::DrawTarget() const
{
	const int slices = 8;
	const tVector offset = tVector(0, 0, 0, 0);
	const double r = gTargetRadius;
	const tVector col0 = tVector(0, 0, 0, 1);
	const tVector col1 = tVector(1, 1, 0, 1);
	const tVector line_col = tVector(0, 0, 0, 1);

	const tVector& target = GetTargetPos();
	
	cDrawUtil::DrawCalibMarker(target + offset, r, slices, col0, col1);
	cDrawUtil::SetLineWidth(1);
	cDrawUtil::DrawCalibMarker(target + offset, r, slices, line_col, line_col, 
								cDrawUtil::eDrawWire);
}

void cScenarioArm::DrawArm(const std::shared_ptr<cSimCharacter>& arm, const tVector& fill_tint, const tVector& line_col) const
{
	cDrawSimCharacter::Draw(*(arm.get()), fill_tint, line_col);

	// draw end effector
	int end_id = cSimArm::eJointLinkEnd;
	tVector end_pos = arm->CalcJointPos(end_id);
	tVector axis;
	double theta;
	arm->CalcJointWorldRotation(end_id - 1, axis, theta);
	
	tVector col = arm->GetPartColor(end_id);

	glPushMatrix();
	cDrawUtil::Translate(end_pos);
	cDrawUtil::Rotate(theta, axis);
	cDrawUtil::SetColor(fill_tint.cwiseProduct(col));
	cDrawUtil::DrawBox(tVector::Zero(), tVector(0.1, 0.1, 0.12, 0), cDrawUtil::eDrawSolid);
	cDrawUtil::SetColor(line_col);
	cDrawUtil::DrawBox(tVector::Zero(), tVector(0.1, 0.1, 0.12, 0), cDrawUtil::eDrawWire);
	glPopMatrix();
}

const std::unique_ptr<cTextureDesc>& cScenarioArm::GetViewRT() const
{
	return mRenderTarget;
}

std::string cScenarioArm::GetName() const
{
	return "Arm";
}

void cScenarioArm::BuildWorld()
{
	cScenarioSimChar::BuildWorld();
	mWorld->SetLinearDamping(gLinearDamping);
	mWorld->SetAngularDamping(gAngularDamping);
}

bool cScenarioArm::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cArmNNController> student_ctrl;
	if (mStudentType == eStudentNN)
	{
		std::shared_ptr<cArmNNController> ctrl = std::shared_ptr<cArmNNController>(new cArmNNController());
		ctrl->Init(mChar.get());
		student_ctrl = ctrl;
	}
	else if (mStudentType == eStudentPDNN)
	{
		std::shared_ptr<cArmPDNNController> ctrl = std::shared_ptr<cArmPDNNController>(new cArmPDNNController());
		ctrl->Init(mChar.get(), mGravity, mCharacterFile);
		student_ctrl = ctrl;
	}
	else if (mStudentType == eStudentNNPixel)
	{
		std::shared_ptr<cArmNNPixelController> ctrl = std::shared_ptr<cArmNNPixelController>(new cArmNNPixelController());
		ctrl->Init(mChar.get());
		student_ctrl = ctrl;
	}
	else
	{
		assert(false); // unsupported character type
	}

	student_ctrl->SetTorqueLimit(gTorqueLim);
	student_ctrl->SetUpdatePeriod(gCtrlUpdatePeriod);

	if (succ && mNetFile != "")
	{
		succ &= student_ctrl->LoadNet(mNetFile);

		if (succ && mModelFiles.size() > 0)
		{
			for (size_t i = 0; i < mModelFiles.size(); ++i)
			{
				student_ctrl->LoadModel(mModelFiles[i]);
			}
		}

		if (succ && mScaleFile != "")
		{
			student_ctrl->LoadScale(mScaleFile);
		}
	}
	
	if (succ)
	{
		out_ctrl = student_ctrl;
	}
	
	return succ;
}

bool cScenarioArm::BuildCoachController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cArmController> coach_ctrl;
	
	if (mCoachType == eCoachQP)
	{
		std::shared_ptr<cArmQPController> ctrl = std::shared_ptr<cArmQPController>(new cArmQPController());
		ctrl->Init(mCoach.get(), mGravity);
		coach_ctrl = ctrl;
	}
	else if (mCoachType == eCoachPDQP)
	{
		std::shared_ptr<cArmPDQPController> ctrl = std::shared_ptr<cArmPDQPController>(new cArmPDQPController());
		ctrl->Init(mCoach.get(), mGravity, mCharacterFile);
		coach_ctrl = ctrl;
	}
	else
	{
		assert(false); // unsupported coach type
	}
	
	coach_ctrl->SetTorqueLimit(gTorqueLim);
	coach_ctrl->SetUpdatePeriod(gCtrlUpdatePeriod);
	out_ctrl = coach_ctrl;

	return succ;
}

void cScenarioArm::BuildGround()
{
}

void cScenarioArm::CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const
{
	out_char = std::shared_ptr<cSimCharacter>(new cSimArm());
}

void cScenarioArm::InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const
{
}

void cScenarioArm::BuildCoach()
{
	CreateCharacter(mCoach);

	cSimCharacter::tParams char_params;
	char_params.mPos = GetDefaultCharPos();
	char_params.mCharFile = mCharacterFile;
	char_params.mStateFile = mCharStateFile;
	char_params.mPlaneCons = GetCharPlaneCons();

	bool succ = mCoach->Init(mWorld, char_params);
	if (succ)
	{
		mCoach->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
		InitCharacterPos(mCoach);

		std::shared_ptr<cCharController> ctrl;
		succ = BuildCoachController(ctrl);
		if (succ)
		{
			mCoach->SetController(ctrl);
		}
	}
}

std::shared_ptr<cArmController> cScenarioArm::GetArmController() const
{
	const auto& ctrl = mChar->GetController();
	if (ctrl == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmController>(ctrl);
}

void cScenarioArm::UpdateCoach(double time_step)
{
	mCoach->Update(time_step);
}

void cScenarioArm::UpdateGround()
{
}

void cScenarioArm::ResetGround()
{
}

void cScenarioArm::SetCtrlTargetPos(const tVector& target)
{
	auto ctrl = GetArmController();
	if (ctrl != nullptr)
	{
		ctrl->SetTargetPos(target);
	}
}

bool cScenarioArm::HasExploded() const
{
	bool exploded = false;
	exploded |= mChar->HasExploded();
	exploded |= mCoach->HasExploded();

	Eigen::VectorXd vel;
	mCoach->BuildVel(vel);
	double hack_test = vel.lpNorm<Eigen::Infinity>();
	if (hack_test > 60)
	{
		exploded = true;
	}

	return exploded;
}

void cScenarioArm::RandReset()
{
	cScenarioSimChar::Reset();
	mCoach->Reset();
	
	ApplyRandPose();
	if (mEnableAutoTarget)
	{
		SetRandTarget();
	}
}

void cScenarioArm::ApplyRandPose()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mCoach->BuildPose(pose);
	mCoach->BuildVel(vel);
	
	int root_size = mCoach->GetParamSize(mCoach->GetRootID());
	for (int i = root_size; i < static_cast<int>(pose.size()); ++i)
	{
		double rand_pose_val = cMathUtil::RandDouble(-M_PI, M_PI);
		double rand_vel_val = 0.2 * cMathUtil::RandDouble(-M_PI, M_PI);
		pose[i] = rand_pose_val;
		vel[i] = rand_vel_val;
	}

	mCoach->SetPose(pose);
	mChar->SetPose(pose);
	mCoach->SetVel(vel);
	mChar->SetVel(vel);
	ResetPoseCounter();
}

void cScenarioArm::SetRandTarget()
{
	tVector target = tVector::Zero();
	target[0] = cMathUtil::RandDouble(-0.5 * gCamSize, 0.5 * gCamSize);
	target[1] = cMathUtil::RandDouble(-0.5 * gCamSize, 0.5 * gCamSize);
	SetTargetPos(target);
	ResetTargetCounter();
}

void cScenarioArm::ResetTargetCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandTargetMinMaxTime(min_time, max_time);
	mTargetCounter = cMathUtil::RandDouble(min_time, max_time);
}

void cScenarioArm::ResetPoseCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandPoseMinMaxTime(min_time, max_time);
	mPoseCounter = cMathUtil::RandDouble(min_time, max_time);
}

void cScenarioArm::UpdateTargetCounter(double time_step)
{
	mTargetCounter -= time_step;
	mTargetCounter = std::max(0.0, mTargetCounter);
	if (mEnableAutoTarget && mTargetCounter <= 0)
	{
		SetRandTarget();
	}
}

void cScenarioArm::UpdatePoseCounter(double time_elapsed)
{
	mPoseCounter -= time_elapsed;
	mPoseCounter = std::max(0.0, mPoseCounter);
	if (mEnableRandPose && mPoseCounter <= 0 && mEnableTraining)
	{
		ApplyRandPose();
	}
}

void cScenarioArm::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	if (new_update)
	{
		UpdateViewBuffer();
		SetNNViewFeatures();
	}

	cScenarioSimChar::UpdateCharacter(time_step);
}

void cScenarioArm::InitViewBuffer()
{
	mViewBuffer = Eigen::VectorXd::Zero(GetStateSize());
}

void cScenarioArm::InitCam()
{
	tVector focus = mChar->GetRootPos();
	tVector pos = focus + tVector(0, 0, 1, 0);
	tVector up = tVector(0, 1, 0, 0);
	const double w = gCamSize;
	const double h = gCamSize;
	const double near_z = 0.01f;
	const double far_z = 5.f;
	mRTCam = cCamera(pos, focus, up, w, h, near_z, far_z);
	mRTCam.SetProj(cCamera::eProjOrtho);
}

bool cScenarioArm::NeedViewBuffer() const
{
	bool need = false;
	auto student = GetArmController();
	
	if (student != nullptr)
	{
		need = (typeid(*student.get()).hash_code() == typeid(cArmNNPixelController).hash_code());
	}
	
	return need;
}

void cScenarioArm::UpdateViewBuffer()
{
	if (NeedViewBuffer())
	{
		mRenderTarget->BindBuffer();
		cDrawUtil::ClearColor(tVector(1, 1, 1, 0));
		cDrawUtil::ClearDepth(1);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		mRTCam.SetupGLProj();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		mRTCam.SetupGLView();

		DrawTarget();
		DrawCharacter();

		mRenderTarget->UnbindBuffer();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		cDrawUtil::Finish();

		mRenderTarget->ReadPixels(mViewBufferRaw);
		int num_texels = mRenderTarget->GetNumTexels();
		int w = mRenderTarget->GetWidth();
		int h = mRenderTarget->GetHeight();

		mViewBuffer.resize(num_texels);

		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				tVector texel = ReadTexel(x, y, w, h, mViewBufferRaw);

				int idx = w * y + x;
				double val = 1 - (texel[0] + texel[1] + texel[2]) / 3;// *texel[3];
				mViewBuffer[idx] = val;
			}
		}
	}
}

void cScenarioArm::SetNNViewFeatures()
{
	auto student = GetStudentController();
	if (typeid(*student.get()).hash_code() == typeid(cArmNNPixelController).hash_code())
	{
		std::shared_ptr<cArmNNPixelController> pixel_ctrl = std::static_pointer_cast<cArmNNPixelController>(student);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
}

void cScenarioArm::InitRenderResources()
{
	mRenderTarget = std::unique_ptr<cTextureDesc>(new cTextureDesc(gRTSize, gRTSize, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));
}

bool cScenarioArm::NeedCtrlUpdate() const
{
	const auto ctrl = GetArmController();
	return ctrl->NeedUpdate();
}

void cScenarioArm::ParseStudent(const cArgParser& parser, eStudent& out_student) const
{
	std::string str = "";
	parser.ParseString("student_type", str);

	if (str == "")
	{
	}
	else if (str == "nn")
	{
		out_student = eStudentNN;
	}
	else if (str == "pd_nn")
	{
		out_student = eStudentPDNN;
	}
	else if (str == "nn_pixel")
	{
		out_student = eStudentNNPixel;
	}
	else
	{
		printf("Unsupported student type %s\n", str.c_str());
		assert(false);
	}
}

void cScenarioArm::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 2;
	out_max = 6;
}

void cScenarioArm::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 6;
	out_max = 8;
}