#include "ScenarioArmRL.h"
#include "stuff/SimArm.h"
#include "render/DrawUtil.h"
#include "render/DrawSimCharacter.h"

const int gTupleBufferSize = 16;
const int gTrainerBufferSize = 20000;
const double gCamSize = 4;
const int gRTSize = 128;

const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const double gTorqueLim = 400;
const double gCtrlUpdatePeriod = 1 / 60.0;

const double gLinearDamping = 0;
const double gAngularDamping = 0;

cScenarioArmRL::cScenarioArmRL()
{
	mEnableTraining = true;
	mSimStepsPerUpdate = 5;
	mTargetPos = tVector(1, 0, 0, 0);
}

cScenarioArmRL::~cScenarioArmRL()
{
}

void cScenarioArmRL::Init()
{
	cScenarioSimChar::Init();
	BuildCoach();

	InitTupleBuffer();
	InitTrainer();
	Reset();

	InitRenderResources();
	InitCam();
}

void cScenarioArmRL::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("solver_file", mSolverFile);
	parser.ParseString("net_file", mNetFile);
	parser.ParseString("model_file", mModelFile);
}

void cScenarioArmRL::Reset()
{
	//cScenarioSimChar::Reset();
	//mCoach->Reset();
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

void cScenarioArmRL::SetTargetPos(const tVector& target)
{
	mTargetPos = target;
	mTargetPos[0] = cMathUtil::Clamp(mTargetPos[0] , -0.5 * gCamSize, gCamSize);
	mTargetPos[1] = cMathUtil::Clamp(mTargetPos[1], -0.5 * gCamSize, gCamSize);
}

const tVector& cScenarioArmRL::GetTargetPos() const
{
	return mTargetPos;
}

void cScenarioArmRL::DrawCharacter() const
{
	cDrawSimCharacter::Draw(*(mChar.get()), gFillTint, gLineColor);
}

void cScenarioArmRL::DrawTarget() const
{
	const int slices = 8;
	const tVector offset = tVector(0, 0, 0, 0);
	const double r = 0.15;
	const tVector col0 = tVector(0, 0, 0, 1);
	const tVector col1 = tVector(1, 1, 0, 1);
	const tVector line_col = tVector(0, 0, 0, 1);

	const tVector& target = GetTargetPos();
	
	cDrawUtil::DrawCalibMarker(target + offset, r, slices, col0, col1);
	cDrawUtil::SetLineWidth(1);
	cDrawUtil::DrawCalibMarker(target + offset, r, slices, line_col, line_col, 
								cDrawUtil::eDrawWire);
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
	//const cBallController& ctrl = mBall.GetController();
	//ctrl.SaveNet(out_file);
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
}

bool cScenarioArmRL::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cArmNNController> ctrl = std::shared_ptr<cArmNNController>(new cArmNNController());
	ctrl->Init(mChar.get());
	ctrl->SetTorqueLimit(gTorqueLim);
	ctrl->SetUpdatePeriod(gCtrlUpdatePeriod);

	if (succ && mNetFile != "")
	{
		succ &= ctrl->LoadNet(mNetFile);

		if (succ && mModelFile != "")
		{
			ctrl->LoadModel(mModelFile);
		}
	}
	
	if (succ)
	{
		out_ctrl = ctrl;
	}
	
	return succ;
}

bool cScenarioArmRL::BuildCoachController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;
	std::shared_ptr<cArmQPController> ctrl = std::shared_ptr<cArmQPController>(new cArmQPController());
	ctrl->Init(mCoach.get(), gGravity);
	ctrl->SetTorqueLimit(gTorqueLim);
	ctrl->SetUpdatePeriod(gCtrlUpdatePeriod);
	out_ctrl = ctrl;

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
	auto ctrl = GetCoachController();
	ctrl->SetTargetPos(target);
}

bool cScenarioArmRL::HasExploded() const
{
	bool exploded = false;
	exploded |= mChar->HasExploded();
	exploded |= mCoach->HasExploded();
	return exploded;
}

void cScenarioArmRL::RandReset()
{
	cScenarioSimChar::Reset();
	mCoach->Reset();
	ApplyRandPose();
	SetRandTarget();
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
		double rand_vel_val = cMathUtil::RandDouble(-M_PI, M_PI) * 0.5;
		pose[i] = rand_pose_val;
		vel[i] = rand_vel_val;
	}

	mCoach->SetPose(pose);
	mChar->SetPose(pose);
	mCoach->SetVel(vel);
	mChar->SetVel(vel);
}

void cScenarioArmRL::SetRandTarget()
{
	tVector target = tVector::Zero();
	target[0] = cMathUtil::RandDouble(-0.25 * gCamSize, 0.25 * gCamSize);
	target[1] = cMathUtil::RandDouble(-0.25 * gCamSize, 0.25 * gCamSize);
	SetTargetPos(target);
}


int cScenarioArmRL::GetStateSize() const
{
	auto ctrl = GetCoachController();
	return ctrl->GetPoliStateSize();
}

int cScenarioArmRL::GetActionSize() const
{
	auto ctrl = GetCoachController();
	return ctrl->GetPoliActionSize();
}

void cScenarioArmRL::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetCoachController();
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
	tuple.mStateEnd = tuple.mStateBeg; // just a place holder
	++mNumTuples;
}

void cScenarioArmRL::UpdateCharacter(double time_step)
{
	bool new_update = NeedCtrlUpdate();
	if (new_update)
	{
		UpdateViewBuffer();
	}
	
	UpdateCoach(time_step);

	// hack
	//cScenarioSimChar::UpdateCharacter(time_step);

	if (new_update)
	{
		RecordTuple();

		if (mNumTuples == static_cast<int>(mTupleBuffer.size()))
		{
			Train();
		}
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
	int net_pool_size = 2; // double Q learning
	//mTrainer.Init(mNetFile, mSolverFile, gTrainerBufferSize, net_pool_size);

	if (mModelFile != "")
	{
		//mTrainer.LoadModel(mModelFile);
	}
}

void cScenarioArmRL::Train()
{
	int iter = GetIter();
	printf("\nTraining iter: %i\n", iter);

	const int num_steps = 1;

	mNumTuples = 0;

	/*
	mTrainer.AddTuples(mTupleBuffer);
	mTrainer.Train(num_steps);

	const cNeuralNet& trainer_net = mTrainer.GetNet();
	std::shared_ptr<cArmNNController> ctrl = std::static_pointer_cast<cArmNNController>(mChar->GetController());
	ctrl->CopyNet(trainer_net);
	*/
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

void cScenarioArmRL::UpdateViewBuffer()
{
	mRenderTarget->BindBuffer();
	cDrawUtil::ClearColor(tVector(1, 1, 1, 1));
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
}

void cScenarioArmRL::InitRenderResources()
{
	mRenderTarget = std::unique_ptr<cTextureDesc>(new cTextureDesc(gRTSize, gRTSize, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, false));
}

bool cScenarioArmRL::NeedCtrlUpdate() const
{
	const auto ctrl = GetCoachController();
	return ctrl->NeedUpdate();
}

std::shared_ptr<cArmQPController> cScenarioArmRL::GetCoachController() const
{
	return std::static_pointer_cast<cArmQPController>(mCoach->GetController());;
}