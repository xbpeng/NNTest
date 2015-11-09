#pragma once
#include <memory>

#include "scenarios/DrawScenario.h"
#include "scenarios/ScenarioBallRL.h"
#include "util/CircularBuffer.h"

class cDrawScenarioBallRL : public cDrawScenario
{
public:
	cDrawScenarioBallRL(cCamera& cam);
	virtual ~cDrawScenarioBallRL();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Update(double time_elapsed);
	virtual void Init();
	virtual void Reset();

	virtual std::string BuildTextInfoStr() const;

	std::string GetName() const;

protected:
	cArgParser mArgParser;
	std::shared_ptr<cScenarioBallRL> mScene;
	bool mTrackCharacter;
	std::string mOutputNetFile;
	bool mEnableTrace;
	cCircularBuffer<tVector, Eigen::aligned_allocator<tVector>> mTraceBuffer;

	virtual void BuildScene();

	virtual void UpdateCamera();
	virtual void ToggleTraining();
	virtual void ToggleTrace();
	virtual void UpdateTrace();

	virtual void SaveNet(const std::string& out_file) const;

	virtual void DrawScene();
	virtual void DrawGrid() const;
	virtual void DrawGround() const;
	virtual void DrawTrace() const;
	virtual void DrawBall() const;
};