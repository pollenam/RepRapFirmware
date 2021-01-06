/*
 * InvertedDeltaKinematics.cpp
 *
 *  Created on: 28 nov. 2019
 *      Author: bf
 */

#include <Movement/Kinematics/InvertedDeltaKinematics.h>

#include "Movement/Move.h"
#include "RepRap.h"
#include "Storage/FileStore.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include <Math/Deviation.h>


InvertedDeltaKinematics::InvertedDeltaKinematics(): m_homed_height(300) {
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
//	alwaysReachableHeight = maxPrintingHeight;
//	for (size_t axis = 0; axis < numTowers; ++axis)
//	{
//		homedCarriageHeights[axis] = m_homed_height + endstopAdjustments[axis];
//	}

//	printRadiusSquared = fsquare(printRadius);

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
			m_homed_height = gb.GetFValue();
		}
	}
	break;
	default:
		break;
	}
	return LinearDeltaKinematics::Configure(mCode, gb, reply, error);
}

float InvertedDeltaKinematics::Transform(const float headPos[], size_t axis) const noexcept
{
	//debugPrintf("axis: %d, value: %f\n", axis, headPos[axis]);
	if (axis < numTowers)
	{
		return sqrtf(D2[axis] - fsquare(-headPos[X_AXIS] - towerX[axis]) - fsquare(headPos[Y_AXIS] - towerY[axis]))
			 - headPos[Z_AXIS]
			 + (-headPos[X_AXIS] * xTilt)
			 + (headPos[Y_AXIS] * yTilt);
	}
	else
	{
		return headPos[axis];
	}
}

// Calculate the Cartesian coordinates from the motor coordinates
void InvertedDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[XYZ_AXES]) const noexcept
{
	LinearDeltaKinematics::ForwardTransform(Ha, Hb, Hc, machinePos);
	machinePos[X_AXIS] = -machinePos[X_AXIS];
//	machinePos[Y_AXIS] = machinePos[Y_AXIS];
	machinePos[Z_AXIS] = -machinePos[Z_AXIS];
}

floatc_t InvertedDeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const noexcept
{
	return -LinearDeltaKinematics::ComputeDerivative( deriv, ha, hb, hc);
}

bool InvertedDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	//debugPrintf("CartesianToMotorSteps !!!\n");
	bool ok = true;
		for (size_t axis = 0; axis < numTowers; ++axis)
		{
			const float pos = Transform(machinePos, axis);
			if (std::isnan(pos) || std::isinf(pos))
			{
				ok = false;
			}
			else
			{
				motorPos[axis] = lrintf(pos * stepsPerMm[axis]);
			}
		}

		// Transform any additional axes linearly
		for (size_t axis = numTowers; axis < numVisibleAxes; ++axis)
		{
			motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
		}

		if (reprap.Debug(moduleMove))
		{
			debugPrintf("CartesianToMotorSteps LinearDelta) :");
			for (size_t i = 0; i < numVisibleAxes; ++i)
			{
				debugPrintf(" %.2f / %d / %.2f", machinePos[i], motorPos[i], motorPos[i]/stepsPerMm[i]);
			}
			debugPrintf("\n");
		}
		return ok;
}

void InvertedDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	ForwardTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = numTowers; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

InvertedDeltaKinematics::~InvertedDeltaKinematics() {
	// TODO Auto-generated destructor stub
}

