#include "ScenarioArmRL.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"
#include "util/FileUtil.h"

const int gTupleBufferSize = 32;
const int gTrainerPlaybackMemSize = 50000;

const double gCamSize = 4;
const int gRTSize = 128;
const double gTargetRadius = 0.15;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const double gTorqueLim = 300;
const double gCtrlUpdatePeriod = 1 / 120.0;

const double gLinearDamping = 0;
const double gAngularDamping = 0;

const tVector gTestGravity = tVector::Zero();

cScenarioArmRL::cScenarioArmRL()
{
	mEnableTraining = true;
	mEnableAutoTarget = true;
	mEnableRandPose = true;
	mPretrain = false;
	mSimStepsPerUpdate = 5;
	mTargetPos = tVector(1, 0, 0, 0);
	ResetTargetCounter();
	ResetPoseCounter();

	mCoachType = eCoachQP;
	mStudentType = eStudentNN;
}

cScenarioArmRL::~cScenarioArmRL()
{
}

void cScenarioArmRL::Init()
{
	cScenarioSimChar::Init();
	BuildCoach();

	InitTrainer();
	InitTupleBuffer();
	Reset();

	InitRenderResources();
	InitCam();
	ResetTargetCounter();
	ResetPoseCounter();
}

void cScenarioArmRL::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseString("model_file", mModelFile);
	parser.ParseBool("arm_pretrain", mPretrain);

	ParseCoach(parser, mCoachType);
	ParseStudent(parser, mStudentType);
}

void cScenarioArmRL::Reset()
{
	//cScenarioSimChar::Reset();
	//mCoach->Reset();
	ResetTargetCounter();
	RandReset();
}

void cScenarioArmRL::Clear()
{
	cScenarioSimChar::Clear();
	mRenderTarget.reset();
	mCoach->Clear();
	mNumTuples = 0;
}

void cScenarioArmRL::Update(double time_elapsed)
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

void cScenarioArmRL::ToggleTraining()
{
	mEnableTraining = !mEnableTraining;
}

bool cScenarioArmRL::EnableTraining() const
{
	return mEnableTraining;
}

bool cScenarioArmRL::EnabledAutoTarget() const
{
	return mEnableAutoTarget;
}

void cScenarioArmRL::EnableAutoTarget(bool enable)
{
	mEnableAutoTarget = enable;
}

bool cScenarioArmRL::EnabledRandPose() const
{
	return mEnableRandPose;
}

void cScenarioArmRL::EnableRandPose(bool enable)
{
	mEnableRandPose = enable;
}

void cScenarioArmRL::SetTargetPos(const tVector& target)
{
	mTargetPos = target;
	double min_dist = -0.5 * gCamSize + gTargetRadius;
	double max_dist = 0.5 * gCamSize - gTargetRadius;
	mTargetPos[0] = cMathUtil::Clamp(mTargetPos[0], min_dist, max_dist);
	mTargetPos[1] = cMathUtil::Clamp(mTargetPos[1], min_dist, max_dist);
	ResetTargetCounter();
}

const tVector& cScenarioArmRL::GetTargetPos() const
{
	return mTargetPos;
}

void cScenarioArmRL::DrawCharacter() const
{
	DrawArm(mChar, gFillTint, gLineColor);
}

void cScenarioArmRL::DrawTarget() const
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

void cScenarioArmRL::DrawArm(const std::shared_ptr<cSimCharacter>& arm, const tVector& fill_tint, const tVector& line_col) const
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

const std::unique_ptr<cTextureDesc>& cScenarioArmRL::GetViewRT() const
{
	return mRenderTarget;
}

const std::shared_ptr<cSimCharacter>& cScenarioArmRL::GetCoach() const
{
	return mCoach;
}

void cScenarioArmRL::SaveNet(const std::string& out_file) const
{
	mTrainer.OutputModel(out_file);
}

std::string cScenarioArmRL::GetName() const
{
	return "Arm RL";
}

void cScenarioArmRL::BuildWorld()
{
	cScenarioSimChar::BuildWorld();
	mWorld->SetLinearDamping(gLinearDamping);
	mWorld->SetAngularDamping(gAngularDamping);
	mWorld->SetGravity(gTestGravity);
}

bool cScenarioArmRL::BuildController(std::shared_ptr<cCharController>& out_ctrl)
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
		ctrl->Init(mChar.get(), gTestGravity, mCharacterFile);
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

		if (succ && mModelFile != "")
		{
			student_ctrl->LoadModel(mModelFile);
		}
	}
	
	if (succ)
	{
		out_ctrl = student_ctrl;
	}
	
	return succ;
}

bool cScenarioArmRL::BuildCoachController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cArmController> coach_ctrl;
	
	if (mCoachType == eCoachQP)
	{
		std::shared_ptr<cArmQPController> ctrl = std::shared_ptr<cArmQPController>(new cArmQPController());
		ctrl->Init(mCoach.get(), gTestGravity);
		coach_ctrl = ctrl;
	}
	else if (mCoachType == eCoachPDQP)
	{
		std::shared_ptr<cArmPDQPController> ctrl = std::shared_ptr<cArmPDQPController>(new cArmPDQPController());
		ctrl->Init(mCoach.get(), gTestGravity, mCharacterFile);
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

void cScenarioArmRL::BuildGround()
{
}

void cScenarioArmRL::CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const
{
	out_char = std::shared_ptr<cSimCharacter>(new cSimArm());
}

void cScenarioArmRL::InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const
{
}

void cScenarioArmRL::BuildCoach()
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

void cScenarioArmRL::UpdateCoach(double time_step)
{
	mCoach->Update(time_step);
}

void cScenarioArmRL::UpdateGround()
{
}

void cScenarioArmRL::ResetGround()
{
}

void cScenarioArmRL::SetCtrlTargetPos(const tVector& target)
{
	auto coach = GetCoachController();
	auto student = GetStudentController();

	if (coach != nullptr)
	{
		coach->SetTargetPos(target);
	}
	if (student != nullptr)
	{
		student->SetTargetPos(target);
	}
}

bool cScenarioArmRL::HasExploded() const
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

void cScenarioArmRL::RandReset()
{
	cScenarioSimChar::Reset();
	mCoach->Reset();
	
	ApplyRandPose();
	if (mEnableAutoTarget)
	{
		SetRandTarget();
	}
}

void cScenarioArmRL::ApplyRandPose()
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

void cScenarioArmRL::SetRandTarget()
{
	tVector target = tVector::Zero();
	target[0] = cMathUtil::RandDouble(-0.5 * gCamSize, 0.5 * gCamSize);
	target[1] = cMathUtil::RandDouble(-0.5 * gCamSize, 0.5 * gCamSize);
	SetTargetPos(target);
	ResetTargetCounter();
}

void cScenarioArmRL::ResetTargetCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandTargetMinMaxTime(min_time, max_time);
	mTargetCounter = cMathUtil::RandDouble(min_time, max_time);
}

void cScenarioArmRL::ResetPoseCounter()
{
	double min_time = 0;
	double max_time = 0;
	GetRandPoseMinMaxTime(min_time, max_time);
	mPoseCounter = cMathUtil::RandDouble(min_time, max_time);
}

void cScenarioArmRL::UpdateTargetCounter(double time_step)
{
	mTargetCounter -= time_step;
	mTargetCounter = std::max(0.0, mTargetCounter);
	if (mEnableAutoTarget && mTargetCounter <= 0)
	{
		SetRandTarget();
	}
}

void cScenarioArmRL::UpdatePoseCounter(double time_elapsed)
{
	mPoseCounter -= time_elapsed;
	mPoseCounter = std::max(0.0, mPoseCounter);
	if (mEnableRandPose && mPoseCounter <= 0 && mEnableTraining)
	{
		ApplyRandPose();
	}
}

int cScenarioArmRL::GetStateSize() const
{
	return mTrainer.GetStateSize();
}

int cScenarioArmRL::GetActionSize() const
{
	return mTrainer.GetActionSize();
}

void cScenarioArmRL::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetStudentController();
	ctrl->RecordPoliState(out_state);
}

void cScenarioArmRL::RecordAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetCoachController();
	ctrl->RecordPoliAction(out_action);
}

void cScenarioArmRL::RecordTuple()
{
	tExpTuple& tuple = mTupleBuffer[mNumTuples];
	RecordState(tuple.mStateBeg);
	RecordAction(tuple.mAction);
	++mNumTuples;
}

void cScenarioArmRL::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	if (new_update)
	{
		UpdateViewBuffer();
		SetNNViewFeatures();
	}
	
	if (EnableSyncCharacters())
	{
		SyncCharacters();
	}

	cScenarioSimChar::UpdateCharacter(time_step);
	UpdateCoach(time_step);

	if (new_update && mEnableTraining)
	{
		bool exploded = HasExploded();
		if (!exploded)
		{
			RecordTuple();

			if (mNumTuples >= static_cast<int>(mTupleBuffer.size()))
			{
				Train();
			}
		}
	}

	if (new_update)
	{
		PrintInfo();
	}
}

void cScenarioArmRL::InitTupleBuffer()
{
	mTupleBuffer.resize(gTupleBufferSize);
	for (int i = 0; i < gTupleBufferSize; ++i)
	{
		mTupleBuffer[i] = tExpTuple(GetStateSize(), GetActionSize());
	}
}

void cScenarioArmRL::InitTrainer()
{
	cNeuralNetTrainer::tParams params;
	params.mNetFile = mNetFile;
	params.mSolverFile = mSolverFile;
	params.mPlaybackMemSize = gTrainerPlaybackMemSize;
	params.mPoolSize = 1;
	params.mNumInitSamples = 10000;
	//params.mNumInitSamples = 100;
	params.mCalcScale = false;
	mTrainer.Init(params);

	SetupScale();

	if (mModelFile != "")
	{
		mTrainer.LoadModel(mModelFile);
	}
}

void cScenarioArmRL::SetupScale()
{
	int state_size = mTrainer.GetStateSize();
	if (state_size > 0)
	{
		auto ctrl = GetStudentController();
		Eigen::VectorXd mean = Eigen::VectorXd::Zero(state_size);
		Eigen::VectorXd stdev = Eigen::VectorXd::Ones(state_size);
		ctrl->BuildPoliStateScale(mean, stdev);
		mTrainer.SetScale(mean, stdev);
	}
}

void cScenarioArmRL::Train()
{
	int iter = GetIter();
	int num_tuples = mTrainer.GetNumTuples();
	printf("\nTraining Iter: %i\n", iter);
	printf("Num Tuples: %i\n", num_tuples);

	const int num_steps = 1;
	mTrainer.AddTuples(mTupleBuffer);

	cNeuralNetTrainer::eStage stage0 = mTrainer.GetStage();
	mTrainer.Train(num_steps);
	cNeuralNetTrainer::eStage stage1 = mTrainer.GetStage();

	const cNeuralNet& trainer_net = mTrainer.GetNet();
	std::shared_ptr<cArmNNController> ctrl = GetStudentController();
	if (ctrl != nullptr)
	{
		ctrl->CopyNet(trainer_net);
	}

	mNumTuples = 0;
}

int cScenarioArmRL::GetIter() const
{
	return mTrainer.GetIter();
}

void cScenarioArmRL::InitCam()
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

bool cScenarioArmRL::NeedViewBuffer() const
{
	bool need = false;
	auto student = GetStudentController();
	
	if (student != nullptr)
	{
		need = (typeid(*student.get()).hash_code() == typeid(cArmNNPixelController).hash_code());
	}
	
	return need;
}

void cScenarioArmRL::UpdateViewBuffer()
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

		DrawCharacter();
		DrawTarget();

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

		//num_texels /= 4;
		//w /= 2;
		//h /= 2;

		mViewBuffer.resize(num_texels);

		//FILE* hack_f = cFileUtil::OpenFile("output/tex_data.txt", "w");
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				//int tex_x = 2 * x;
				//int tex_y = 2 * y;

				//tVector texel0 = ReadTexel(tex_x, tex_y, w * 2, h * 2, mViewBufferRaw);
				//tVector texel1 = ReadTexel(tex_x + 1, tex_y, w * 2, h * 2, mViewBufferRaw);
				//tVector texel2 = ReadTexel(tex_x, tex_y + 1, w * 2, h * 2, mViewBufferRaw);
				//tVector texel3 = ReadTexel(tex_x + 1, tex_y + 1, w * 2, h * 2, mViewBufferRaw);
				//tVector texel = (texel0 + texel1 + texel2 + texel3) / 4;

				tVector texel = ReadTexel(x, y, w, h, mViewBufferRaw);

				int idx = w * y + x;
				// hack
				double val = 1 - (texel[0] + texel[1] + texel[2]) / 3;// *texel[3];
				mViewBuffer[idx] = val;

				//fprintf(hack_f, "%.5f\t", val);
			}
			//fprintf(hack_f, "\n");
		}
		//cFileUtil::CloseFile(hack_f);
	}
}

void cScenarioArmRL::SetNNViewFeatures()
{
	auto student = GetStudentController();
	if (typeid(*student.get()).hash_code() == typeid(cArmNNPixelController).hash_code())
	{
		std::shared_ptr<cArmNNPixelController> pixel_ctrl = std::static_pointer_cast<cArmNNPixelController>(student);
		pixel_ctrl->SetViewBuffer(mViewBuffer);
	}
}

void cScenarioArmRL::InitRenderResources()
{
	mRenderTarget = std::unique_ptr<cTextureDesc>(new cTextureDesc(gRTSize, gRTSize, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));
}

bool cScenarioArmRL::NeedCtrlUpdate() const
{
	const auto ctrl = GetStudentController();
	return ctrl->NeedUpdate();
}

std::shared_ptr<cArmQPController> cScenarioArmRL::GetCoachController() const
{
	const auto& coach = mCoach->GetController();
	if (coach == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmQPController>(coach);
}

std::shared_ptr<cArmNNController> cScenarioArmRL::GetStudentController() const
{
	const auto& student = mChar->GetController();
	if (student == nullptr)
	{
		return nullptr;
	}
	return std::static_pointer_cast<cArmNNController>(student);
}

void cScenarioArmRL::SyncCharacters()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	
	cNeuralNetTrainer::eStage trainer_stage = mTrainer.GetStage();
	bool sync_student = mPretrain || trainer_stage == cNeuralNetTrainer::eStageInit;

	if (sync_student)
	{
		mCoach->BuildPose(pose);
		mCoach->BuildVel(vel);
		mChar->SetPose(pose);
		mChar->SetVel(vel);
	}
	else
	{
		mChar->BuildPose(pose);
		mChar->BuildVel(vel);
		mCoach->SetPose(pose);
		mCoach->SetVel(vel);
	}
}

bool cScenarioArmRL::EnableSyncCharacters() const
{
	return mEnableTraining;
}

void cScenarioArmRL::ParseCoach(const cArgParser& parser, eCoach& out_coach) const
{
	std::string str = "";
	parser.ParseString("coach_type", str);

	if (str == "")
	{
	}
	else if (str == "qp")
	{
		out_coach = eCoachQP;
	}
	else if (str == "pd_qp")
	{
		out_coach = eCoachPDQP;
	}
	else
	{
		printf("Unsupported coach type %s\n", str.c_str());
		assert(false);
	}
}

void cScenarioArmRL::ParseStudent(const cArgParser& parser, eStudent& out_student) const
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

void cScenarioArmRL::PrintInfo() const
{
	Eigen::VectorXd coach_action;
	Eigen::VectorXd student_action;
	auto coach = GetCoachController();
	auto student = GetStudentController();
	coach->RecordPoliAction(coach_action);
	student->RecordPoliAction(student_action);

	printf("Coach Action: ");
	for (int i = 0; i < coach_action.size(); ++i)
	{
		printf("%.3f\t", coach_action[i]);
	}
	printf("\n");

	printf("Student Action: ");
	for (int i = 0; i < student_action.size(); ++i)
	{
		printf("%.3f\t", student_action[i]);
	}
	printf("\n");

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	printf("Student Pose: ");
	for (int i = 3; i < pose.size(); ++i)
	{
		printf("%.3f\t", pose[i]);
	}
	printf("\n");

	printf("Student Vel: ");
	for (int i = 3; i < vel.size(); ++i)
	{
		printf("%.3f\t", vel[i]);
	}
	printf("\n");

	printf("\n");
}

void cScenarioArmRL::GetRandTargetMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 2;
	out_max = 6;
}

void cScenarioArmRL::GetRandPoseMinMaxTime(double& out_min, double& out_max) const
{
	out_min = 6;
	out_max = 8;
}