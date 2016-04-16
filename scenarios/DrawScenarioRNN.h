#pragma once
#include <memory>

#include "scenarios/DrawScenario.h"
#include "ScenarioRNN.h"

class cDrawScenarioRNN : public cDrawScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioRNN(cCamera& cam);
	virtual ~cDrawScenarioRNN();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);

	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, int x, int y);

	std::string GetName() const;

protected:
	cArgParser mArgParser;
	std::unique_ptr<cScenarioRNN> mScene;
	tVector mMousePos;
	bool mMousePressed;
	bool mAutoTrainer;
	cRand mRand;

	virtual void BuildScene(std::unique_ptr<cScenarioRNN>& out_scene);
	virtual void DrawScene();
	virtual void DrawPoints() const;
	virtual void DrawNetEval() const;
	virtual void TrainNet();
};