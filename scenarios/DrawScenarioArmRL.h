#pragma once
#include <memory>

#include "scenarios/DrawScenarioArm.h"
#include "scenarios/ScenarioArmRL.h"
#include "util/CircularBuffer.h"
#include "sim/CharTracer.h"

class cDrawScenarioArmRL : public cDrawScenarioArm
{
public:
	cDrawScenarioArmRL(cCamera& cam);
	virtual ~cDrawScenarioArmRL();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Update(double time_elapsed);

protected:
	std::string mOutputNetFile;
	bool mOutputTorques;
	FILE* mCoachTorqueFile;
	FILE* mStudentTorqueFile;

	virtual void ToggleTraining();
	virtual std::shared_ptr<cScenarioArmRL> GetRLScene() const;
	virtual void BuildScene();

	virtual void InitTracer();
	virtual void SaveNet(const std::string& out_file) const;

	virtual void DrawCharacter();

	virtual void ToggleOutputTorques();
	virtual void BeginWrite();
	virtual void EndWrite();
	virtual void WriteTorques();
	virtual void WriteTorques(const std::shared_ptr<cSimCharacter>& character, FILE* out_file) const;

	virtual void ToggleOutputData();
};