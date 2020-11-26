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
	const char *GetName(bool forStatusReport) const noexcept override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;

	virtual ~InvertedDeltaKinematics();

protected:
	void Recalc() noexcept override;
    float Transform(const float headPos[], size_t axis) const noexcept override;								// Calculate the motor position for a single tower from a Cartesian coordinate
    void ForwardTransform(float Ha, float Hb, float Hc, float machinePos[XYZ_AXES]) const noexcept override;
    floatc_t ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const noexcept override;

private:
	float m_homed_height; // height above carriage when homed.

};

#endif /* SRC_MOVEMENT_KINEMATICS_INVERTEDDELTAKINEMATICS_H_ */
