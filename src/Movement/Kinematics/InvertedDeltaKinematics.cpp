/*
 * InvertedDeltaKinematics.cpp
 *
 *  Created on: 28 nov. 2019
 *      Author: bf
 */

#include <Movement/Kinematics/InvertedDeltaKinematics.h>

InvertedDeltaKinematics::InvertedDeltaKinematics(): belowHomedHeight(300) {
	// TODO Auto-generated constructor stub
}

void InvertedDeltaKinematics::Recalc()
{
	LinearDeltaKinematics::Recalc();

//	// Calculate the squares of the diagonals and the base carriage heights when the printer is homed, i.e. the carriages are at the endstops
//	// Also calculate the always-reachable height
//	alwaysReachableHeight = homedHeight + belowHomedHeight;
//	debugPrintf("Ben : setting alwaysReachableHeight to Homing Height : %f \n", homedHeight);
//	for (size_t axis = 0; axis < numTowers; ++axis)
//	{
//		homedCarriageHeights[axis] = homedHeight
//									+ sqrtf(D2[axis] - ((axis < UsualNumTowers) ? fsquare(radius) : fsquare(towerX[axis]) + fsquare(towerY[axis])))
//									+ endstopAdjustments[axis];
//		const float heightLimit = homedCarriageHeights[axis] - diagonals[axis];
//		if (heightLimit < alwaysReachableHeight)
//		{
//			alwaysReachableHeight = heightLimit;
//			debugPrintf("Ben : setting alwaysReachableHeight to Height Limit : %f \n", heightLimit);
//		}
//	}
//
//	printRadiusSquared = fsquare(printRadius);
//
//	if (reprap.Debug(moduleMove))
//	{
//		debugPrintf("HCH:");
//		for (size_t i = 0; i < numTowers; ++i)
//		{
//			debugPrintf(" %.2f", (double)homedCarriageHeights[i]);
//		}
//		debugPrintf("\n");
//	}
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
LimitPositionResult InvertedDeltaKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const
{
	return LimitPositionResult::ok;

//	constexpr AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
//	bool limited = false;
//
//	// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
//	// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
//	if ((axesHomed & allAxes) == allAxes)
//	{
//		// Constrain the move to be within the build radius
//		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
//		if (diagonalSquared > printRadiusSquared)
//		{
//			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
//			finalCoords[X_AXIS] *= factor;
//			finalCoords[Y_AXIS] *= factor;
//			limited = true;
//		}
//
//		// Constrain the position to be within the reachable height
//		if (initialCoords == nullptr)
//		{
//			// Asking to limit a single position
//			if (finalCoords[Z_AXIS] > alwaysReachableHeight)
//			{
//				for (size_t tower = 0; tower < UsualNumTowers; ++tower)
//				{
//					const float carriageHeight = Transform(finalCoords, tower);
//					debugPrintf("Ben : Carriage Height %f\n", carriageHeight);
//					if (carriageHeight > homedCarriageHeights[tower])
//					{
//						finalCoords[Z_AXIS] -= (carriageHeight - homedCarriageHeights[tower]);
//						limited = true;
//					}
//				}
//			}
//
//		}
//		else if (finalCoords[Z_AXIS] > alwaysReachableHeight || initialCoords[Z_AXIS] > alwaysReachableHeight)
//		{
//			debugPrintf("Ben : finalCoords[Z_AXIS] (%f) > alwaysReachableHeight || initialCoords[Z_AXIS] (%f) > alwaysReachableHeight - alwaysReachableHaigth = %f\n",finalCoords[Z_AXIS], initialCoords[Z_AXIS], alwaysReachableHeight);
//
//			// Asking to limit all positions along a straight line
//			// Determine the maximum reachable height at the final position and all intermediate positions
//			const float dx = finalCoords[X_AXIS] - initialCoords[X_AXIS],
//						dy = finalCoords[Y_AXIS] - initialCoords[Y_AXIS];
//			const float P2 = fsquare(dx) + fsquare(dy);							// square of the distance moved in the XY plane
//			float dz = finalCoords[Z_AXIS] - initialCoords[Z_AXIS];
//			float Q2 = P2 + fsquare(dz);										// square of the total distance moved
//			if (Q2 != 0.0)														// if there is any XYZ movement
//			{
//				// If t is the proportion of movement completed from initial to final coordinates, the t-value corresponding to the maximum tower height is:
//				// t = (+/- dz*sqrt(d^2*P2 - (dx*(y0-yt)-dy*(x0-xt))^2)*Q
//			    //      -(x0-xt)*dx*Q2
//			    //      -(y0-yt)*dy*Q2
//				//     )
//				//	   /(P2*Q2)
//				// We want the root that increases with increasing dz, i.e. positive Z movement delays the maximum
//				for (size_t tower = 0; tower < numTowers; ++tower)
//				{
//					const float tx = initialCoords[X_AXIS] - towerX[tower],
//								ty = initialCoords[Y_AXIS] - towerY[tower];
//					const float discriminant = (D2[tower] * P2) - fsquare((dx * ty) - (dy * tx));
//					bool limitFinalHeight;
//					bool again;													// we may need to iterate
//					do
//					{
//						again = false;
//						if (discriminant < 0.0)
//						{
//							// There is no maximum carriage height on this tower, so the maximum must occur at the initial or final point.
//							// We assume that the initial point is within range, so check the final point.
//							limitFinalHeight = true;
//						}
//						else
//						{
//							const float tP2Q2 = dz * sqrtf(discriminant * Q2) - ((tx * dx) + (ty * dy)) * Q2;
//							const float P2Q2 = P2 * Q2;
//							if (tP2Q2 >= P2Q2)
//							{
//								limitFinalHeight = true;						// the maximum is beyond the final position
//							}
//							else
//							{
//								limitFinalHeight = false;
//								if (tP2Q2 > 0.0)
//								{
//									const float t = tP2Q2/P2Q2;
//									float tempCoords[XYZ_AXES];
//									tempCoords[X_AXIS] = initialCoords[X_AXIS] + t * dx;
//									tempCoords[Y_AXIS] = initialCoords[Y_AXIS] + t * dy;
//									tempCoords[Z_AXIS] = initialCoords[Z_AXIS] + t * dz;
//									const float carriageHeight = Transform(tempCoords, tower);
//
//									if (carriageHeight > homedCarriageHeights[tower])
//									{
//										// We can't do this move as requested
//										const float proposedAdjustment = carriageHeight - homedCarriageHeights[tower] + 0.5;
//										if (dz >= proposedAdjustment)
//										{
//											// There is some chance that if we reduce the requested final Z coordinate, we can do the move
//											finalCoords[Z_AXIS] -= proposedAdjustment;
//											dz -= proposedAdjustment;;
//											limited = true;
//
//											// Update the intermediate variables that have changed
//											again = true;
//											Q2 = P2 + fsquare(dz);
//											if (reprap.Debug(moduleMove))
//											{
//												debugPrintf("Limit tower %u, t=%.2f\n", tower, (double)t);
//											}
//										}
//										else
//										{
//											return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
//										}
//									}
//								}
//							}
//						}
//					} while (again);
//
//					if (limitFinalHeight)
//					{
//						debugPrintf("Ben : limit final height");
//						const float carriageHeight = Transform(finalCoords, tower);
//						if (carriageHeight > homedCarriageHeights[tower])
//						{
//							const float proposedAdjustment = carriageHeight - homedCarriageHeights[tower];
//							if (dz >= proposedAdjustment)
//							{
//								finalCoords[Z_AXIS] -= proposedAdjustment;
//								limited = true;
//								if (reprap.Debug(moduleMove))
//								{
//									debugPrintf("Limit tower %u\n", tower);
//								}
//								if (tower + 1 < numTowers)
//								{
//									dz -= proposedAdjustment;
//									Q2 = P2 + fsquare(dz);
//								}
//							}
//							else
//							{
//								return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
//							}
//						}
//					}
//				}
//			}
//		}
//
//		if (applyM208Limits && finalCoords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
//		{
//			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
//			limited = true;
//		}
//	}
//
//	// Limit any additional axes according to the M208 limits
//	if (applyM208Limits && LimitPositionFromAxis(finalCoords, numTowers, numVisibleAxes, axesHomed))
//	{
//		limited = true;
//	}
//
//	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}



bool InvertedDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	debugPrintf("Hi Guys\n");
	float invertedMachinePos[MaxAxes];
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		invertedMachinePos[axis] = machinePos[axis];
	}
	invertedMachinePos[Z_AXIS] = -machinePos[Z_AXIS];
	debugPrintf("inverted : %f, before : %f \n", invertedMachinePos[Z_AXIS], machinePos[Z_AXIS]);
	return LinearDeltaKinematics::CartesianToMotorSteps(invertedMachinePos, stepsPerMm, numVisibleAxes, numTotalAxes, motorPos, isCoordinated);
}

void InvertedDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	debugPrintf("Hi Guys2\n");
	LinearDeltaKinematics::MotorStepsToCartesian(motorPos, stepsPerMm, numVisibleAxes, numTotalAxes, machinePos);
	machinePos[Z_AXIS] = -machinePos[Z_AXIS];
}

InvertedDeltaKinematics::~InvertedDeltaKinematics() {
	// TODO Auto-generated destructor stub
}

