#pragma once
#include <memory>

#include "scenarios/DrawScenarioSimInteractive.h"
#include "scenarios/ScenarioArmRL.h"
#include "util/CircularBuffer.h"

class cDrawScenarioArmRL : public cDrawScenario
{
public:
	cDrawScenarioArmRL(cCamera& cam);
	virtual ~cDrawScenarioArmRL();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void Update(double time_elapsed);
	virtual void Init();
	virtual void Reset();

	virtual std::string BuildTextInfoStr() const;
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

	std::string GetName() const;

protected:
	cArgParser mArgParser;
	std::shared_ptr<cScenarioArmRL> mScene;
	std::shared_ptr<cScenarioSimChar> mSimScene;
	std::string mOutputNetFile;
	bool mMouseDown;

	virtual void ToggleTraining();
	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const;
	virtual void BuildScene();

	virtual void SetTarget(const tVector& target);

	virtual void SaveNet(const std::string& out_file) const;

	virtual void DrawScene();
	virtual void DrawCharacter();
	virtual void DrawTarget() const;
	virtual void DrawGrid() const;
	virtual void DrawViewRT() const;
};