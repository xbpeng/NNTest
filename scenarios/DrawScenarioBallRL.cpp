#include "DrawScenarioBallRL.h"
#include "render/DrawUtil.h"
#include "stuff/DrawPenaltyGround.h"
#include "stuff/DrawBall.h"

const tVector gCamPos0 = tVector(0, 0.75, 1, 0);
const int gTraceSize = 1000;

cDrawScenarioBallRL::cDrawScenarioBallRL(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslatePos(gCamPos0);
	mTrackCharacter = false;
	mEnableTrace = true;
	mOutputNetFile = "";
	mTraceBuffer.Reserve(gTraceSize);
}

cDrawScenarioBallRL::~cDrawScenarioBallRL()
{
}

void cDrawScenarioBallRL::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenario::ParseArgs(parser);
	mArgParser = parser;

	parser->ParseString("output_net_file", mOutputNetFile);
}

void cDrawScenarioBallRL::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenario::Keyboard(key, x, y);

	switch (key)
	{
	case 'c':
		mTrackCharacter = !mTrackCharacter;
		break;
	case 't':
		ToggleTraining();
		break;
	case 's':
		SaveNet();
		break;
	case 'w':
		ToggleTrace();
		break;
	case 'd':
		DrawActionDist(!DrawActionDist());
		break;
	default:
		break;
	}
}

void cDrawScenarioBallRL::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	mScene->Update(time_elapsed);
	UpdateCamera();

	if (mEnableTrace && time_elapsed > 0)
	{
		UpdateTrace();
	}
}

void cDrawScenarioBallRL::Init()
{
	cDrawScenario::Init();
	BuildScene();
	mScene->ParseArgs(mArgParser);
	mScene->Init();
}

void cDrawScenarioBallRL::Reset()
{
	cDrawScenario::Reset();
	mScene->Reset();
	mTraceBuffer.Clear();
}

void cDrawScenarioBallRL::DrawScene()
{
	DrawGrid();
	DrawGround();

	if (mEnableTrace)
	{
		DrawTrace();
	}

	DrawBall();

	if (DrawActionDist())
	{
		DrawActionDistSamples();
	}
}

void cDrawScenarioBallRL::BuildScene()
{
	mScene = std::shared_ptr<cScenarioBallRL>(new cScenarioBallRL());
}

void cDrawScenarioBallRL::UpdateCamera()
{
	tVector char_pos = mScene->GetBallPos();
	tVector cam_focus = mCam.GetFocus();

	if (mTrackCharacter)
	{
		cam_focus[0] = char_pos[0];
		mCam.TranslateFocus(cam_focus);
	}
	else
	{
		double cam_w = mCam.GetWidth();
		cam_w *= 0.5;
		const double pad = std::min(0.5, cam_w);
		if (std::abs(char_pos[0] - cam_focus[0]) > cam_w - pad)
		{
			cam_focus[0] = char_pos[0];
			cam_focus[0] += cam_w - pad;
			mCam.TranslateFocus(cam_focus);
		}
	}
}

void cDrawScenarioBallRL::ToggleTraining()
{
	mScene->ToggleTraining();
	bool enable_training = mScene->EnableTraining();
	if (enable_training)
	{
		printf("Training enabled\n");
	}
	else
	{
		printf("Training disabled\n");
	}
}

void cDrawScenarioBallRL::ToggleTrace()
{
	mEnableTrace = !mEnableTrace;
	mTraceBuffer.Clear();
}

void cDrawScenarioBallRL::UpdateTrace()
{
	const tVector& pos = mScene->GetBallPos();
	mTraceBuffer.Add(pos);
}


void cDrawScenarioBallRL::DrawActionDist(bool enable)
{
	const auto& ctrl = mScene->GetBall().GetController();
	ctrl->RecordActionDist(enable);
}

bool cDrawScenarioBallRL::DrawActionDist() const
{
	const auto& ctrl = mScene->GetBall().GetController();
	return ctrl->RecordActionDist();
}

tVector cDrawScenarioBallRL::GetDefaultCamPos() const
{
	return gCamPos0;
}

tVector cDrawScenarioBallRL::GetCamTrackPos() const
{
	tVector pos = GetDefaultCamPos();
	tVector ball_pos = mScene->GetBallPos();
	pos[0] = ball_pos[0];
	return pos;
}

tVector cDrawScenarioBallRL::GetCamStillPos() const
{
	return GetCamTrackPos();
}

void cDrawScenarioBallRL::SaveNet() const
{
	SaveNet(mOutputNetFile);
}

void cDrawScenarioBallRL::SaveNet(const std::string& out_file) const
{
	mScene->SaveNet(out_file);
	printf("Model saved to %s\n", out_file.c_str());
}

std::string cDrawScenarioBallRL::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioBallRL::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	const tVector& origin = mCam.GetFocus();
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	glColor4f(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f);
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioBallRL::DrawTrace() const
{
	cDrawUtil::SetColor(tVector(0, 0, 1, 0.2));
	cDrawUtil::SetLineWidth(3);

	int num_trace = static_cast<int>(mTraceBuffer.GetSize());
	for (int i = 0; i < num_trace - 1; ++i)
	{
		const tVector& a = mTraceBuffer[i];
		const tVector& b = mTraceBuffer[i + 1];

		cDrawUtil::DrawLine(a, b);
	}
	
}

void cDrawScenarioBallRL::DrawGround() const
{
	const auto& ground = mScene->GetGround();

	tVector focus = mCam.GetFocus();
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();

	tVector bound_min = focus - tVector(cam_w, cam_h, 0, 0) * 0.5;
	tVector bound_max = focus + tVector(cam_w, cam_h, 0, 0) * 0.5;

	cDrawPenaltyGround::Draw(ground, bound_min, bound_max);
}

void cDrawScenarioBallRL::DrawBall() const
{
	const tVector col = tVector(120 / 255.0, 140 / 255.0, 175 / 255.0, 1.0);

	const auto& ball = mScene->GetBall();
	cDrawBall::Draw(ball, col);
}

void cDrawScenarioBallRL::DrawActionDistSamples() const
{
	const int num_bins = 20;

	const auto& ball = mScene->GetBall();
	const auto& ctrl = ball.GetController();
	const Eigen::MatrixXd& samples = ctrl->GetActionDistSamples();
	int num_samples = samples.rows();

	if (num_samples > 0)
	{
		const tVector marker_size = tVector(0, 0.05, 0, 0);
		const tVector& pos_beg = ctrl->GetPosBeg();

		cDrawUtil::SetColor(tVector(0, 0, 1, 0.5));
		cDrawUtil::SetLineWidth(4);

		Eigen::VectorXi bins = Eigen::VectorXi::Zero(num_bins);
		double min_dist = samples.col(0).minCoeff();
		double max_dist = samples.col(0).maxCoeff();

		for (int i = 0; i < num_samples; ++i)
		{
			tVector curr_pos = pos_beg;
			double curr_dist = samples(i, 0);

			int curr_bin = num_bins * (curr_dist - min_dist) / (max_dist - min_dist);
			curr_bin = cMathUtil::Clamp(curr_bin, 0, num_bins - 1);

			++bins[curr_bin];
		}

		double bar_w = (max_dist - min_dist) / num_bins;
		for (int b = 0; b < num_bins; ++b)
		{
			int curr_count = bins[b];
			double bar_h = static_cast<double>(curr_count) / num_samples;
			bar_h *= 2 / (max_dist - min_dist);

			tVector pos = pos_beg;
			pos[0] += bar_w * (b + 0.5) + min_dist;
			pos[1] += 0.5 * bar_h;
			cDrawUtil::DrawRect(pos, tVector(bar_w, bar_h, 0, 0));
		}
	}
}

std::string cDrawScenarioBallRL::BuildTextInfoStr() const
{
	double succ_rate = mScene->GetSuccRate();
	std::string info = "Success Rate: " + std::to_string(succ_rate);
	return info;
}

bool cDrawScenarioBallRL::IsDone() const
{
	return mScene->IsDone();
}