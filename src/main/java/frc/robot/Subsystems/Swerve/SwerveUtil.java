// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;

/** Add your docs here. */
public class SwerveUtil {
    
    public static double nativeToMetersPerSecond(double encoderVelocity, double gearRatio) {
        return encoderVelocity * 10 * getMetersPerCount(gearRatio);
    }

    public static double nativeToRadians(double encoderCounts) {
        return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
    }

    /**
     * 
     * @param metersPerSecond the speed in meters per second to be converted
     * @return the equivalent speed motor encoder velocity in counts per 100 ms
     */
    public double metersPerSecondToNative(double metersPerSecond, double gearRatio) {
        return metersPerSecond / getMetersPerCount(gearRatio) / 10;
    }

    /**
     * 
     * @param radians number of radians to convert
     * @return radians converted to absolute encoder counts
     */
    public static double radiansToNative(double radians) {
        return radians / (2 * Math.PI) * ANGLE_ENCODER_CPR;
    }

    public static double getMetersPerCount(double gearRatio) {
        return WHEEL_CIRCUMFERENCE / gearRatio / TALON_ENCODER_CPR;
    }
}
