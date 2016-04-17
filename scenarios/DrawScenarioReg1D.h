#pragma once
#include <memory>

#include "scenarios/DrawScenario.h"
#include "ScenarioReg1D.h"

class cDrawScenarioReg1D : public cDrawScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioReg1D(cCamera& cam);
	virtual ~cDrawScenarioReg1D();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);

	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

	std::string GetName() const;

protected:
	cArgParser mArgParser;
	std::unique_ptr<cScenarioReg1D> mScene;
	tVector mMousePos;
	bool mMousePressed;
	bool mAutoTrainer;

	virtual void BuildScene(std::unique_ptr<cScenarioReg1D>& out_scene);
	virtual void DrawScene();
	virtual void DrawPoints() const;
	virtual void DrawNetEval() const;
	virtual void TrainNet();

	virtual void OutputPoints() const;
};