#pragma once
#include <memory>

#include "scenarios/DrawScenarioSimInteractive.h"
#include "scenarios/ScenarioArm.h"
#include "util/CircularBuffer.h"
#include "sim/CharTracer.h"

class cDrawScenarioArm : public cDrawScenario
{
public:
	cDrawScenarioArm(cCamera& cam);
	virtual ~cDrawScenarioArm();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Update(double time_elapsed);
	virtual void Init();
	virtual void Clear();
	virtual void Reset();

	virtual bool IsDone() const;

	virtual std::string BuildTextInfoStr() const;
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

	std::string GetName() const;

protected:
	std::shared_ptr<cArgParser> mArgParser;
	std::shared_ptr<cScenarioArm> mScene;
	std::shared_ptr<cScenarioSimChar> mSimScene;
	bool mMouseDown;

	bool mEnableTrace;
	cCharTracer mTracer;
	std::vector<int> mTraceHandles;

	virtual void ToggleAutoTarget();
	virtual void ToggleRandPose();
	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const;
	virtual void BuildScene();

	virtual void InitTracer();
	virtual int AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
								const tVector& col);
	virtual int AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
								const tVectorArr& cols);
	virtual void UpdateTrace(double time_elapsed);
	virtual void ToggleTrace();

	virtual void SetTarget(const tVector& target);

	virtual void ApplyRandForce();
	virtual tVector GetDefaultCamPos() const;
	virtual int GetEndEffectorID() const;

	virtual void DrawScene();
	virtual void DrawCharacter();
	virtual void DrawPerturbs() const;
	virtual void DrawTarget() const;
	virtual void DrawGrid() const;
	virtual void DrawViewRT() const;

	virtual void ResetCallback();
	virtual void PostSubstepCallback(double time_elapsed);

	virtual void ToggleRecordMotion();
};