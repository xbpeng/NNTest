#pragma once
#include <memory>

#include "scenarios/DrawScenarioArm.h"
#include "scenarios/ScenarioArmTrain.h"
#include "util/CircularBuffer.h"
#include "sim/CharTracer.h"

class cDrawScenarioArmTrain : public cDrawScenarioArm
{
public:
	cDrawScenarioArmTrain(cCamera& cam);
	virtual ~cDrawScenarioArmTrain();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Update(double time_elapsed);

protected:
	std::string mOutputNetFile;
	
	virtual void ToggleTraining();
	virtual std::shared_ptr<cScenarioArmTrain> GetTrainScene() const;
	virtual void BuildScene();

	virtual void SaveNet(const std::string& out_file) const;

	virtual void DrawCharacter();
};