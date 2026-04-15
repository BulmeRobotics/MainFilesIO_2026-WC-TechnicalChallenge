/**
* @name:    Gyro.cpp
* @date:	15.01.2026
* @authors: Florian Wiesner
* @details: .cpp file for the Gyro (IMU)
*/

//----Libraries----
#include "Gyro.h"

#ifdef _MSC_VER
    #pragma region Init //-------------------------------------------------------------------------------------------------------
#endif
ErrorCodes Gyro::Init(void) {
	if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
		#ifdef DEBUG
		Serial.println("Could not find a valid BNO055 sensor!");
		#endif // DEBUG		
		return ErrorCodes::ERROR;
	}
	bno.setExtCrystalUse(true);
	delay(500);
	ResetAllAngles();
	return ErrorCodes::OK;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Angle //------------------------------------------------------------------------------------------------------
#endif
float Gyro::GetAngle(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event);

	if (axis == GyroAxles::Axis_X) {
		float in = event.orientation.x + diff_x; // calculate the angle, diff_x is the offset after a reset
		if (in > 360) return in - 360;
		if (in < 0) return in + 360;
		return in;
	}
	else if (axis == GyroAxles::Axis_Y) {
		float in = event.orientation.y + diff_y;
		if (in > 360) return in - 360;
		if (in < 0) return in + 360;
		return in;
	}
	else if (axis == GyroAxles::Axis_Z) {
		float in = event.orientation.z + diff_z;
		if (in > 360) return in - 360;
		if (in < 0) return in + 360;
		return in;
	}
	return 0;
}

float Gyro::GetAngleAdvanced(float targetAngle, float actualAngle) {
	// Get and constrain values
    targetAngle = constrain(targetAngle, 0, 360);
    data.angle_abs = constrain(actualAngle, 0, 360);

    // calculate angular error (always between -180° and +180°)
    float angleError = targetAngle - data.angle_abs;
    if (angleError > 180) angleError -= 360;
    if (angleError < -180) angleError += 360;

    // store the angular error
    data.angle_error = angleError;

    // set the direction, based on shortest turn
    if (angleError > 0) {
        data.direction_left = true;   // turn left
        data.direction_right = false;
    } else if (angleError < 0) {
        data.direction_left = false;
        data.direction_right = true; // turn right
    } else {
        // if the error is 0, no direction is set
        data.direction_left = false;
        data.direction_right = false;
    }

    // exactly 180°-turn
    if (abs(angleError) == 180) {
        // always left
        data.direction_left = true;
        data.direction_right = false;
    }

    // calculate the cartesian angle (-180° bis +180°)
    if (data.angle_abs <= 180) {
        data.angle_car = data.angle_abs;
    } else {
        data.angle_car = data.angle_abs - 360;
    }

    #ifdef DEBUG_GYRO_ADVANCED
    Serial.print("target: ");
    Serial.print(targetAngle);
    Serial.print("\tabs: ");
    Serial.print(data.angle_abs);
    Serial.print("\tcar: ");
    Serial.print(data.angle_car);
    Serial.print("\terror: ");
    Serial.print(data.angle_error);
    Serial.print("\tleft: ");
    Serial.print(data.direction_left);
    Serial.print("\tright: ");
    Serial.println(data.direction_right);
	#endif

    // return the absolute angle
    return data.angle_abs;
}

float Gyro::GetAngleAdvanced(float targetAngle, GyroAxles axis) {
    return GetAngleAdvanced(targetAngle, GetAngle(axis));
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Orientation //------------------------------------------------------------------------------------------------
#endif
float Gyro::GetAngleFromOrientation(Orientations orientation) {
	switch (orientation) {
	case Orientations::North: return 0;
	case Orientations::East: return 90;
	case Orientations::South: return 180;
	case Orientations::West: return 270;
	default: return 0;
	}
}

Orientations Gyro::GetOrientationFromAngle(float angle) {
    angle = constrain(angle, 0, 360);   // constrain input
	if (angle >= 315 || angle < 45) return Orientations::North;
	else if (angle >= 45 && angle < 135) return Orientations::East;
	else if (angle >= 135 && angle < 225) return Orientations::South;
	else if (angle >= 225 && angle < 315) return Orientations::West;
	return Orientations::North;
}

Orientations Gyro::GetOrientationFromAngle(void) {  // overloaded
	float angle = GetAngle(GyroAxles::Axis_X);
	return GetOrientationFromAngle(angle);
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Accel & Gravity //--------------------------------------------------------------------------------------------
#endif
float Gyro::GetAcceleration(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

	if (axis == GyroAxles::Axis_X)	return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y)	return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z)	return event.acceleration.z;
	return 0;
}

float Gyro::GetGravity(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);

	if (axis == GyroAxles::Axis_X)	return event.acceleration.x;
	else if (axis == GyroAxles::Axis_Y)	return event.acceleration.y;
	else if (axis == GyroAxles::Axis_Z)	return event.acceleration.z;
	return 0;
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Reset //------------------------------------------------------------------------------------------------------
#endif
void Gyro::ResetAngle(GyroAxles axis) {
	if (axis == GyroAxles::Axis_X) diff_x = -GetRawAngle(GyroAxles::Axis_X);
	else if (axis == GyroAxles::Axis_Y) diff_y = -GetRawAngle(GyroAxles::Axis_Y);
	else if (axis == GyroAxles::Axis_Z) diff_z = -GetRawAngle(GyroAxles::Axis_Z);
}

void Gyro::ResetAllAngles(void) {
	ResetAngle(GyroAxles::Axis_X);
	ResetAngle(GyroAxles::Axis_Y);
	ResetAngle(GyroAxles::Axis_Z);
}

void Gyro::SetAngle(GyroAxles axis, float value) {
    value = constrain(value, 0, 360);   //constrain input

	if (axis == GyroAxles::Axis_X) {
		ResetAngle(axis);
		diff_x = diff_x + value;
		if (diff_x > 360) diff_x -= 360;
		if (diff_x < 0) diff_x += 360;
	}
	else if (axis == GyroAxles::Axis_Y) {
		ResetAngle(axis);
		diff_y = diff_y + value;
		if (diff_y > 360) diff_y -= 360;
		if (diff_y < 0) diff_y += 360;
	}
	else if (axis == GyroAxles::Axis_Z) {
		ResetAngle(axis);
		diff_z = diff_z + value;
		if (diff_z > 360) diff_z -= 360;
		if (diff_z < 0) diff_z += 360;
	}
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Temp //-------------------------------------------------------------------------------------------------------
#endif
int8_t Gyro::GetTemp(void) {
	return bno.getTemp();
}

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Private //----------------------------------------------------------------------------------------------------
#endif
//Private method to get the raw angle measurement from the sensor; NOT optimal to get the angle for the main program
float Gyro::GetRawAngle(GyroAxles axis) {
	sensors_event_t event;
	bno.getEvent(&event);

	if (axis == GyroAxles::Axis_X)	return  event.orientation.x;
	else if (axis == GyroAxles::Axis_Y)	return event.orientation.y;
	else if (axis == GyroAxles::Axis_Z)	return event.orientation.z;
	return 0;
}

#ifdef _MSC_VER
    #pragma endregion
#endif