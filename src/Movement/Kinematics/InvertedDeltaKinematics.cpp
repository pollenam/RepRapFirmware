/*
 * InvertedDeltaKinematics.cpp
 *
 *  Created on: 28 nov. 2019
 *      Author: bf
 */

#include <Movement/Kinematics/InvertedDeltaKinematics.h>
#include "RepRap.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"


InvertedDeltaKinematics::InvertedDeltaKinematics(): maxPrintingHeight(300) {
	// TODO Auto-generated constructor stub
}

// Return the name of the current kinematics
const char *InvertedDeltaKinematics::GetName(bool forStatusReport) const noexcept
{
	return (forStatusReport) ? "idelta" : "Inverted Linear delta";
}

void InvertedDeltaKinematics::Recalc() noexcept
{
	LinearDeltaKinematics::Recalc();

	// Calculate the squares of the diagonals and the base carriage heights when the printer is homed, i.e. the carriages are at the endstops
	// Also calculate the always-reachable height
	alwaysReachableHeight = maxPrintingHeight;
	for (size_t axis = 0; axis < numTowers; ++axis)
	{
		homedCarriageHeights[axis] = -homedHeight
									+ sqrtf(D2[axis] - ((axis < UsualNumTowers) ? fsquare(radius) : fsquare(towerX[axis]) + fsquare(towerY[axis])))
									+ endstopAdjustments[axis];
	}

	printRadiusSquared = fsquare(printRadius);

	if (reprap.Debug(moduleMove))
	{
		debugPrintf("HCH - Inverted :");
		for (size_t i = 0; i < numTowers; ++i)
		{
			debugPrintf(" %.2f", (double)homedCarriageHeights[i]);
		}
		debugPrintf("\n");
	}
}

bool InvertedDeltaKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	switch(mCode)
	{
	case 665:
	{
		if (gb.Seen('J'))
		{
			maxPrintingHeight = gb.GetFValue();
		}
	}
	break;
	default:
		break;
	}
	return LinearDeltaKinematics::Configure(mCode, gb, reply, error);
}

bool InvertedDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	// float invertedMachinePos[MaxAxes];
	// for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	// {
	// 	invertedMachinePos[axis] = machinePos[axis];
	// }
	// invertedMachinePos[Z_AXIS] = -machinePos[Z_AXIS];
	bool r = LinearDeltaKinematics::CartesianToMotorSteps(machinePos, stepsPerMm, numVisibleAxes, numTotalAxes, motorPos, isCoordinated);
//	if (reprap.Debug(moduleMove))
//	{
//		debugPrintf("CartesianToMotorSteps :");
//		for (size_t i = 0; i < numVisibleAxes; ++i)
//		{
//			debugPrintf(" %.2f / %d", invertedMachinePos[i], motorPos[i]);
//		}
//		debugPrintf("\n");
//	}
	return r;
}

void InvertedDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	return LinearDeltaKinematics::MotorStepsToCartesian(motorPos, stepsPerMm, numVisibleAxes, numTotalAxes, machinePos);
	// machinePos[Z_AXIS] = -machinePos[Z_AXIS];
	// if (reprap.Debug(moduleMove))
	// {
	// 	debugPrintf("MotorStepsToCartesian :");
	// 	for (size_t i = 0; i < numVisibleAxes; ++i)
	// 	{
	// 		debugPrintf(" %.2f / %d", machinePos[i], motorPos[i]);
	// 	}
	// 	debugPrintf("\n");
	// }
}

LimitPositionResult InvertedDeltaKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept
{
	finalCoords[Z_AXIS] = -finalCoords[Z_AXIS];
	LimitPositionResult r = LinearDeltaKinematics::LimitPosition(finalCoords, initialCoords, numVisibleAxes, axesHomed, isCoordinated, applyM208Limits);
	finalCoords[Z_AXIS] = -finalCoords[Z_AXIS];
	return r;
}

InvertedDeltaKinematics::~InvertedDeltaKinematics() {
	// TODO Auto-generated destructor stub
}

