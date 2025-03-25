#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>
#include "MotorControl.h"

enum VerticalState
{
	VERTICAL_UNKNOWN,
	VERTICAL_VERTEX,
	VERTICAL_EDGE,
	VERTICAL_NONE
};

namespace StateManager
{
	void initializeState();
	void handleCurrentState();
	void handleCalibrationIndication();
	void indicateCalibration();
	void updateVerticalState();
	void handleVerticalVertexState();
	void handleVerticalEdgeState();
}

#endif // STATE_MANAGER_H
