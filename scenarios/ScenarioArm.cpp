#include "ScenarioArm.h"
#include "stuff/SimArm.h"
#include "stuff/ArmQPController.h"
#include "stuff/ArmPDQPController.h"
#include "stuff/ArmVelQPController.h"
#include "stuff/ArmControllerMACE.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const double gCamSize = 4;
const double gTargetRadius = 0.15;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const double gTorqueLim = 300;
const double gCtrlUpdatePeriod = 1 / 120.0;

const double gLinearDamping = 0;
const double gAngularDamping = 0;

cScenarioArm::cScenarioArm()
{
	mEnableAutoTarget = true;
	mEnableRandPose = true;
	mNumUpdateSteps = 5;
	mTargetPos = tVector(1, 1, 0, 0);
	ResetTargetCounter();
	ResetPoseCounter();

	mCtrlType = eCtrlNone;
	mGravity = tVector(0, 0, 0, 0);

	mRand.Seed(static_cast<unsigned long int>(cMathUtil::RandInt(0, std::numeric_limits<int>::max())));
}

cScenarioArm::~cScenarioArm()
{
}

void cScenarioArm::Init()
{
	cScenarioSimChar::Init();
	InitViewBuffer();

	InitRenderResources();
	InitCam();
	ResetTargetCounter();
	ResetPoseCounter();
}

void cScenarioArm::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("net_file", mNetFile);
	parser.ParseStringArray("model_file", mModelFiles);
	parser.ParseString("scale_file", mScaleFile);

	ParseCtrlType(parser, "arm_ctrl_type", mCtrlType);
}

void cScenarioArm::Reset()
{
	cScenarioSimChar::Reset();
	ResetTargetCounter();
	RandReset();
}

void cScenarioArm::Clear()
{
	cScenarioSimChar::Clear();
	mRenderTarget.reset();
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
	auto arm_ctrl = GetArmController();
	int end_id = arm_ctrl->GetEndEffectorID();
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
	return BuildController(mChar, mCtrlType, out_ctrl);
}

bool cScenarioArm::BuildController(const std::shared_ptr<cSimCharacter>& character, eCtrlType ctrl_type, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cArmController> ctrl = nullptr;
	
	if (ctrl_type == eCtrlNone)
	{
	}
	else if (ctrl_type == eCtrlNN || ctrl_type == eCtrlPDNN || ctrl_type == eCtrlVelNN 
		|| ctrl_type == eCtrlNNPixel || ctrl_type == eCtrlPDNNPixel || ctrl_type == eCtrlVelNNPixel
		|| ctrl_type == eCtrlNNPixelNoPose || ctrl_type == eCtrlMACE)
	{
		succ = BuildNNController(ctrl_type, ctrl);
	}
	else if (ctrl_type == eCtrlQP)
	{
		std::shared_ptr<cArmQPController> curr_ctrl = std::shared_ptr<cArmQPController>(new cArmQPController());
		curr_ctrl->Init(character.get(), mGravity);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlPDQP)
	{
		std::shared_ptr<cArmPDQPController> curr_ctrl = std::shared_ptr<cArmPDQPController>(new cArmPDQPController());
		curr_ctrl->Init(character.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlVelQP)
	{
		std::shared_ptr<cArmVelQPController> curr_ctrl = std::shared_ptr<cArmVelQPController>(new cArmVelQPController());
		curr_ctrl->Init(character.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else
	{
		assert(false); // unsupported character type
	}

	if (ctrl != nullptr)
	{
		ctrl->SetTorqueLimit(gTorqueLim);
		ctrl->SetUpdatePeriod(gCtrlUpdatePeriod);
	}
	
	if (succ)
	{
		out_ctrl = ctrl;
	}

	return succ;
}

bool cScenarioArm::BuildNNController(eCtrlType ctrl_type, std::shared_ptr<cArmController>& out_ctrl)
{
	bool succ = true;

	std::shared_ptr<cArmNNController> ctrl = nullptr;

	if (ctrl_type == eCtrlNN)
	{
		std::shared_ptr<cArmNNController> curr_ctrl = std::shared_ptr<cArmNNController>(new cArmNNController());
		curr_ctrl->Init(mChar.get());
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlPDNN)
	{
		std::shared_ptr<cArmPDNNController> curr_ctrl = std::shared_ptr<cArmPDNNController>(new cArmPDNNController());
		curr_ctrl->Init(mChar.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlVelNN)
	{
		std::shared_ptr<cArmVelNNController> curr_ctrl = std::shared_ptr<cArmVelNNController>(new cArmVelNNController());
		curr_ctrl->Init(mChar.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlNNPixel)
	{
		std::shared_ptr<cArmNNPixelController> curr_ctrl = std::shared_ptr<cArmNNPixelController>(new cArmNNPixelController());
		curr_ctrl->Init(mChar.get());
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlPDNNPixel)
	{
		std::shared_ptr<cArmPDNNPixelController> curr_ctrl = std::shared_ptr<cArmPDNNPixelController>(new cArmPDNNPixelController());
		curr_ctrl->Init(mChar.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlVelNNPixel)
	{
		std::shared_ptr<cArmVelNNPixelController> curr_ctrl = std::shared_ptr<cArmVelNNPixelController>(new cArmVelNNPixelController());
		curr_ctrl->Init(mChar.get(), mGravity, mCharacterFile);
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlNNPixelNoPose)
	{
		std::shared_ptr<cArmNNPixelNoPoseController> curr_ctrl = std::shared_ptr<cArmNNPixelNoPoseController>(new cArmNNPixelNoPoseController());
		curr_ctrl->Init(mChar.get());
		ctrl = curr_ctrl;
	}
	else if (ctrl_type == eCtrlMACE)
	{
		std::shared_ptr<cArmControllerMACE> curr_ctrl = std::shared_ptr<cArmControllerMACE>(new cArmControllerMACE());
		curr_ctrl->Init(mChar.get());
		ctrl = curr_ctrl;
	}
	else
	{
		assert(false); // unsupported character type
	}

	if (succ && mNetFile != "")
	{
		succ &= ctrl->LoadNet(mNetFile);

		if (succ && mModelFiles.size() > 0)
		{
			for (size_t i = 0; i < mModelFiles.size(); ++i)
			{
				ctrl->LoadModel(mModelFiles[i]);
			}
		}

		if (succ && mScaleFile != "")
		{
			ctrl->LoadScale(mScaleFile);
		}
	}

	if (succ)
	{
		out_ctrl = ctrl;
	}

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

std::shared_ptr<cArmController> cScenarioArm::GetArmController() const
{
	const auto& ctrl = mChar->GetController();
	if (ctrl == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmController>(ctrl);
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

	Eigen::VectorXd vel;
	mChar->BuildVel(vel);
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
	
	if (mEnableRandPose)
	{
		ApplyRandPose();
	}

	if (mEnableAutoTarget)
	{
		SetRandTarget();
	}
}

void cScenarioArm::ApplyRandPose()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);
	
	int pose_size = static_cast<int>(pose.size());
	int root_id = mChar->GetRootID();
	int root_size = mChar->GetParamSize(root_id);
	for (int i = root_size; i < pose_size; ++i)
	{
		double rand_pose_val = mRand.RandDouble(-M_PI, M_PI);
		double rand_vel_val = 0.2 * mRand.RandDouble(-M_PI, M_PI);
		pose[i] = rand_pose_val;
		vel[i] = rand_vel_val;
	}

	mChar->SetPose(pose);
	mChar->SetVel(vel);
	ResetPoseCounter();
}

void cScenarioArm::SetRandTarget()
{
	tVector target = tVector::Zero();
	double max_dist = GetRandTargetMaxDist();
	target[0] = mRand.RandDouble(-max_dist, max_dist);
	target[1] = mRand.RandDouble(-max_dist, max_dist);
	SetTargetPos(target);
	ResetTargetCounter();
}

void cScenarioArm::ResetTargetCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandTargetMinMaxTime(min_time, max_time);
	mTargetCounter = mRand.RandDouble(min_time, max_time);
}

void cScenarioArm::ResetPoseCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandPoseMinMaxTime(min_time, max_time);
	mPoseCounter = mRand.RandDouble(min_time, max_time);
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
	if (mEnableRandPose && mPoseCounter <= 0)
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
	int res = cArmNNPixelController::GetViewBufferRes();
	mViewBuffer = Eigen::VectorXd::Zero(res * res);
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
	auto ctrl = GetArmController();
	
	if (ctrl != nullptr)
	{
		need = (typeid(*ctrl.get()).hash_code() == typeid(cArmNNPixelController).hash_code())
			|| (typeid(*ctrl.get()).hash_code() == typeid(cArmPDNNPixelController).hash_code())
			|| (typeid(*ctrl.get()).hash_code() == typeid(cArmVelNNPixelController).hash_code())
			|| (typeid(*ctrl.get()).hash_code() == typeid(cArmNNPixelNoPoseController).hash_code());
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
	auto ctrl = GetArmController();
	if (typeid(*ctrl.get()).hash_code() == typeid(cArmNNPixelController).hash_code())
	{
		std::shared_ptr<cArmNNPixelController> pixel_ctrl = std::static_pointer_cast<cArmNNPixelController>(ctrl);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
	else if (typeid(*ctrl.get()).hash_code() == typeid(cArmPDNNPixelController).hash_code())
	{
		std::shared_ptr<cArmPDNNPixelController> pixel_ctrl = std::static_pointer_cast<cArmPDNNPixelController>(ctrl);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
	else if (typeid(*ctrl.get()).hash_code() == typeid(cArmVelNNPixelController).hash_code())
	{
		std::shared_ptr<cArmVelNNPixelController> pixel_ctrl = std::static_pointer_cast<cArmVelNNPixelController>(ctrl);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
	else if (typeid(*ctrl.get()).hash_code() == typeid(cArmNNPixelNoPoseController).hash_code())
	{
		std::shared_ptr<cArmNNPixelNoPoseController> pixel_ctrl = std::static_pointer_cast<cArmNNPixelNoPoseController>(ctrl);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
}

void cScenarioArm::InitRenderResources()
{
	int res = cArmNNPixelController::GetViewBufferRes();
	mRenderTarget = std::unique_ptr<cTextureDesc>(new cTextureDesc(res, res, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));
}

bool cScenarioArm::NeedCtrlUpdate() const
{
	const auto ctrl = GetArmController();
	if (ctrl != nullptr)
	{
		return ctrl->NeedUpdate();
	}
	return false;
}

void cScenarioArm::ParseCtrlType(const cArgParser& parser, const std::string& key, eCtrlType& out_ctrl) const
{
	std::string str = "";
	parser.ParseString(key, str);

	if (str == "")
	{
	}
	else if (str == "qp")
	{
		out_ctrl = eCtrlQP;
	}
	else if (str == "pd_qp")
	{
		out_ctrl = eCtrlPDQP;
	}
	else if (str == "vel_qp")
	{
		out_ctrl = eCtrlVelQP;
	}
	else if (str == "nn")
	{
		out_ctrl = eCtrlNN;
	}
	else if (str == "mace")
	{
		out_ctrl = eCtrlMACE;
	}
	else if (str == "pd_nn")
	{
		out_ctrl = eCtrlPDNN;
	}
	else if (str == "vel_nn")
	{
		out_ctrl = eCtrlVelNN;
	}
	else if (str == "nn_pixel")
	{
		out_ctrl = eCtrlNNPixel;
	}
	else if (str == "pd_nn_pixel")
	{
		out_ctrl = eCtrlPDNNPixel;
	}
	else if (str == "vel_nn_pixel")
	{
		out_ctrl = eCtrlVelNNPixel;
	}
	else if (str == "nn_pixel_no_pose")
	{
		out_ctrl = eCtrlNNPixelNoPose;
	}
	else
	{
		printf("Unsupported arm control type %s\n", str.c_str());
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

double cScenarioArm::GetRandTargetMaxDist() const
{
	return 0.5 * gCamSize;
}

int cScenarioArm::GetEndEffectorID() const
{
	auto ctrl = GetArmController();
	return ctrl->GetEndEffectorID();
}