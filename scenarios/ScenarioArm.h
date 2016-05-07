#pragma once

#include <string>
#include "util/MathUtil.h"
#include "util/ArgParser.h"
#include "scenarios/ScenarioSimChar.h"
#include "stuff/ArmController.h"
#include "stuff/ArmNNController.h"
#include "stuff/ArmPDNNController.h"
#include "stuff/ArmVelNNController.h"
#include "stuff/ArmNNPixelController.h"
#include "stuff/ArmPDNNPixelController.h"
#include "stuff/ArmVelNNPixelController.h"
#include "stuff/ArmNNPixelNoPoseController.h"

#include "render/TextureDesc.h"
#include "render/camera.h"

class cScenarioArm : public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioArm();
	virtual ~cScenarioArm();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual bool EnabledAutoTarget() const;
	virtual void EnableAutoTarget(bool enable);

	virtual bool EnabledRandPose() const;
	virtual void EnableRandPose(bool enable);

	virtual void SetTargetPos(const tVector& target);
	virtual const tVector& GetTargetPos() const;

	virtual void DrawCharacter() const;
	virtual void DrawTarget() const;
	virtual void DrawArm(const std::shared_ptr<cSimCharacter>& arm, const tVector& fill_tint, const tVector& line_col) const;
	
	virtual const std::unique_ptr<cTextureDesc>& GetViewRT() const;
	virtual int GetEndEffectorID() const;

	virtual std::string GetName() const;

protected:
	enum eCtrlType
	{
		eCtrlNone,
		eCtrlQP,
		eCtrlPDQP,
		eCtrlVelQP,
		eCtrlNN,
		eCtrlPDNN,
		eCtrlVelNN,
		eCtrlNNPixel,
		eCtrlPDNNPixel,
		eCtrlVelNNPixel,
		eCtrlNNPixelNoPose,
		eCtrlMACE,
		eCtrlPDMACE,
		eCtrlNNTrack,
		eCtrlMax
	};

	bool mEnableAutoTarget;
	bool mEnableRandPose;

	eCtrlType mCtrlType;

	double mPoseCounter;
	double mTargetCounter;

	std::string mNetFile;
	std::vector<std::string> mModelFiles;
	std::string mScaleFile;

	tVector mTargetPos;

	cCamera mRTCam;
	std::unique_ptr<cTextureDesc> mRenderTarget;
	Eigen::VectorXd mViewBuffer;
	std::vector<GLubyte> mViewBufferRaw;
	
	cRand mRand;

	virtual void BuildWorld();
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildController(cTerrainRLCtrlFactory::eCharCtrl char_ctrl, std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildController(const std::shared_ptr<cSimCharacter>& character, eCtrlType ctrl_type, std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildNNController(eCtrlType ctrl_type, std::shared_ptr<cArmController>& out_ctrl);
	
	virtual void BuildGround();

	virtual void CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const;

	virtual std::shared_ptr<cArmController> GetArmController() const;

	virtual void UpdateGround();
	virtual void ResetGround();
	virtual void SetCtrlTargetPos(const tVector& target);

	virtual bool HasExploded() const;
	virtual void RandReset();
	virtual void ApplyRandPose();
	virtual void SetRandTarget();
	virtual void ResetTargetCounter();
	virtual void ResetPoseCounter();
	virtual void UpdateTargetCounter(double time_step);
	virtual void UpdatePoseCounter(double time_elapsed);

	virtual void UpdateCharacter(double time_step);
	virtual void InitViewBuffer();

	virtual void InitCam();
	virtual bool NeedViewBuffer() const;
	virtual void UpdateViewBuffer();
	virtual void SetNNViewFeatures();
	virtual void InitRenderResources();
	virtual bool NeedCtrlUpdate() const;

	virtual void ParseCtrlType(const cArgParser& parser, const std::string& key, eCtrlType& out_ctrl) const;
	
	virtual void GetRandTargetMinMaxTime(double& out_min, double& out_max) const;
	virtual void GetRandPoseMinMaxTime(double& out_min, double& out_max) const;
	virtual double GetRandTargetMaxDist() const;

};