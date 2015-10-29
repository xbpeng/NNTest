#include <caffe/caffe.hpp>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GL/GL.h>

#include "render/Camera.h"
#include "render/DrawUtil.h"
#include "util/FileUtil.h"
#include "util/ArgParser.h"
#include "scenarios/DrawScenario.h"
#include "scenarios/DrawScenarioReg1D.h"
#include "scenarios/DrawScenarioReg1DTrainer.h"
#include "scenarios/DrawScenarioBallRL.h"
#include "scenarios/DrawScenarioArmRL.h"

// Dimensions of the window we are drawing into.
int winWidth = 800;
int winHeight = static_cast<int>(winWidth * 9.f / 16.f);

const tVector gBKGColor = tVector(0.95f, 0.95f, 0.97f, 0.f);

// camera attributes
double gViewWidth = 3.5f;
double gViewHeight = (gViewWidth * winHeight) / winWidth;
double gViewNearZ = 0.01f;
double gViewFarZ = 5.f;

tVector gCameraPosition = tVector(0, 0, 1, 0);
tVector gCameraFocus = tVector(gCameraPosition[0], gCameraPosition[1], 0.f, 0.f);
tVector gCameraUp = tVector(0, 1, 0, 0);

cCamera gCamera;

// anim
const double gFPS = 30.f;
const double gAnimStep = 1.f / gFPS;
const int gDisplayAnimTime = static_cast<int>(gAnimStep * 1000);
bool gAnimate = true;

int gPrevTime = 0;
int gNextTime = 0;
int gDispalyPrevTime = 0;

double gPlaybackSpeed = 1;
const double gPlaybackDelta = 0.1;

// arg parser
cArgParser gArgParser;
std::shared_ptr<cDrawScenario> gScenario = NULL;
int gArgc = 0;

char** gArgv = NULL;

void SetupCamProjection()
{
	gCamera.SetupGLProj();
}

void ResizeCamera()
{
	double cam_h = gCamera.GetHeight();
	double cam_w = (cam_h * winWidth) / winHeight;
	gCamera.Resize(cam_w, cam_h);
}

void InitCamera()
{
	gCamera = cCamera(gCameraPosition, gCameraFocus, gCameraUp,
		gViewWidth, gViewHeight, gViewNearZ, gViewFarZ);
	gCamera.SetProj(cCamera::eProjOrtho);
	ResizeCamera();
}

void ClearScenario()
{
	gScenario = NULL;
}

void SetupScenario()
{
	InitCamera();
	ClearScenario();
	
	std::string scenario_name = "";
	gArgParser.ParseString("scenario", scenario_name);

	if (scenario_name == "reg_1d")
	{
		gScenario = std::shared_ptr<cDrawScenarioReg1D>(new cDrawScenarioReg1D(gCamera));
	}
	else if (scenario_name == "reg_1d_trainer")
	{
		gScenario = std::shared_ptr<cDrawScenarioReg1DTrainer>(new cDrawScenarioReg1DTrainer(gCamera));
	}
	else if (scenario_name == "ball_rl")
	{
		gScenario = std::shared_ptr<cDrawScenarioBallRL>(new cDrawScenarioBallRL(gCamera));
	}
	else if (scenario_name == "arm_rl")
	{
		gScenario = std::shared_ptr<cDrawScenarioArmRL>(new cDrawScenarioArmRL(gCamera));
	}

	if (gScenario != NULL)
	{
		gScenario->ParseArgs(gArgParser);
		gScenario->Init();
		printf("Loaded scenario: %s\n", gScenario->GetName().c_str());
	}
}

void UpdateScenario(double sec_elapsed)
{
	if (gScenario != NULL)
	{
		gScenario->Update(sec_elapsed);
	}
}

void DrawInfo()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const double aspect = gCamera.GetAspectRatio();
	const double text_size = 0.09;
	const tVector scale = tVector(text_size / aspect, text_size, 1, 0);
	const double line_offset = text_size;

	cDrawUtil::SetLineWidth(1.5);
	cDrawUtil::SetColor(tVector(0, 0, 0, 0.5));

	cDrawUtil::Translate(tVector(-0.96, 0.88, -1, 0));

	char buffer[128];
	sprintf_s(buffer, "Playback Speed: %.2fx\n", gPlaybackSpeed);

	std::string text_info = std::string(buffer);
	if (gScenario != nullptr)
	{
		text_info += gScenario->BuildTextInfoStr();
	}

	cDrawUtil::DrawString(text_info.c_str(), scale);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void Display(void)
{
	cDrawUtil::ClearColor(gBKGColor);
	cDrawUtil::ClearDepth(1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (gScenario != NULL)
	{
		gScenario->Draw();
	}

	DrawInfo();

	glutSwapBuffers();

	gDispalyPrevTime = glutGet(GLUT_ELAPSED_TIME);
}

void Reshape(int w, int h)
{
	winWidth = w;
	winHeight = h;

	glViewport(0, 0, winWidth, winHeight);

	UpdateScenario(0);
	ResizeCamera();

	glMatrixMode(GL_PROJECTION);
	SetupCamProjection();

	glutPostRedisplay();
}

void StepAnim(double time_step)
{
	UpdateScenario(time_step);
	gAnimate = false;
	glutPostRedisplay();
}

void ParseArgs(int argc, char** argv)
{
	gArgParser = cArgParser(argv, argc);

	std::string arg_file = "";
	gArgParser.ParseString("arg_file", arg_file);
	if (arg_file != "")
	{
		// append the args from the file to the ones from the commandline
		// this allows the cmd args to overwrite the file args
		gArgParser.AppendArgs(arg_file);
	}
}


void Reload()
{
	ParseArgs(gArgc, gArgv);
	SetupScenario();
}

void Reset()
{
	if (gScenario != NULL)
	{
		gScenario->Reset();
	}
}

void Update(double time_elapsed)
{
	UpdateScenario(time_elapsed);
}

void Animate(int callback_val)
{
	if (gAnimate)
	{
		unsigned int timer_step = static_cast<unsigned int>(gDisplayAnimTime / gPlaybackSpeed);
		timer_step = (std::abs(gPlaybackSpeed) < 0.000001) ? 1000000 : timer_step;
		timer_step = std::abs(static_cast<int>(timer_step));

		glutTimerFunc(timer_step, Animate, 0);

		double time_step = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
		Update(time_step);

		glutPostRedisplay();
	}
}


void ToggleAnimate()
{
	gAnimate = !gAnimate;
	if (gAnimate)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void ChangePlaybackSpeed(double delta)
{
	double prev_playback = gPlaybackSpeed;
	gPlaybackSpeed += delta;

	if (std::abs(prev_playback) < 0.0001 && std::abs(gPlaybackSpeed) > 0.0001)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void Keyboard(unsigned char key, int x, int y) {

	if (gScenario != NULL)
	{
		gScenario->Keyboard(key, x, y);
	}

	switch (key) {
		// Quit.
	case 27: // escape
		exit(0);
		break;
	case '>':
		StepAnim(gAnimStep);
		break;
	case '<':
		StepAnim(-gAnimStep);
		break;
	case 'r':
		Reset();
		break;
	case 'l':
		Reload();
		break;
	case ' ':
		ToggleAnimate();
		break;
	case ',':
		ChangePlaybackSpeed(-gPlaybackDelta);
		break;
	case '.':
		ChangePlaybackSpeed(gPlaybackDelta);
		break;
	case '/':
		gPlaybackSpeed = 1;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

void MouseClick(int button, int state, int x, int y)
{
	double screen_x = static_cast<double>(x) / winWidth;
	double screen_y = static_cast<double>(y) / winHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseClick(button, state, screen_x, screen_y);
	}
}

void MouseMove(int x, int y)
{
	double screen_x = static_cast<double>(x) / winWidth;
	double screen_y = static_cast<double>(y) / winHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseMove(screen_x, screen_y);
	}
}

void InitOpenGl(void)
{
	glEnable(GL_TEXTURE_2D);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	cDrawUtil::InitDrawUtil();
	glewInit();
}

int main(int argc, char** argv)
{
	FLAGS_alsologtostderr = 1;
	int caffe_argc = 1; // hack
	caffe::GlobalInit(&caffe_argc, &argv);

	glutInit(&gArgc, gArgv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(winWidth, winHeight);
	glutCreateWindow("Neural Network Test");

	InitOpenGl();

	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	SetupScenario();
	
	Reshape(winWidth, winHeight);
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove);
	glutTimerFunc(gDisplayAnimTime, Animate, 0);

	glutMainLoop();

	return 0;
}