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
	float invertedMachinePos[MaxAxes];
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		invertedMachinePos[axis] = machinePos[axis];
	}
	invertedMachinePos[Z_AXIS] = -machinePos[Z_AXIS];
	return LinearDeltaKinematics::CartesianToMotorSteps(invertedMachinePos, stepsPerMm, numVisibleAxes, numTotalAxes, motorPos, isCoordinated);
}

void InvertedDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	LinearDeltaKinematics::MotorStepsToCartesian(motorPos, stepsPerMm, numVisibleAxes, numTotalAxes, machinePos);
	machinePos[Z_AXIS] = -machinePos[Z_AXIS];
}

InvertedDeltaKinematics::~InvertedDeltaKinematics() {
	// TODO Auto-generated destructor stub
}

