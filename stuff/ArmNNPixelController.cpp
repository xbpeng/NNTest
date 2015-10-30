#include "ArmNNPixelController.h"
#include "SimArm.h"

const int gViewBufferRes = 128;

cArmNNPixelController::cArmNNPixelController()
{
	mViewBuffer = Eigen::VectorXd::Zero(gViewBufferRes * gViewBufferRes);
}

cArmNNPixelController::~cArmNNPixelController()
{
}

void cArmNNPixelController::SetViewBuffer(const Eigen::VectorXd& view_buff)
{
	assert(view_buff.size() == mViewBuffer.size());
	mViewBuffer = view_buff;
}

void cArmNNPixelController::UpdatePoliState()
{
	cArmNNController::UpdatePoliState();
}