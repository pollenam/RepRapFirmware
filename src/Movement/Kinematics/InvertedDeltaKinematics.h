/*
 * InvertedDeltaKinematics.h
 *
 *  Created on: 28 nov. 2019
 *      Author: bf
 */

#ifndef SRC_MOVEMENT_KINEMATICS_INVERTEDDELTAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_INVERTEDDELTAKINEMATICS_H_

#include "LinearDeltaKinematics.h"

class InvertedDeltaKinematics : public LinearDeltaKinematics {
public:
	InvertedDeltaKinematics();
	virtual LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const;
	virtual bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	virtual void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	//virtual bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error); /*override*/

	virtual ~InvertedDeltaKinematics();

protected:
	virtual void Recalc();

private:
	float belowHomedHeight; // absolute height below homing position

};

#endif /* SRC_MOVEMENT_KINEMATICS_INVERTEDDELTAKINEMATICS_H_ */
