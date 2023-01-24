// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import static frc.robot.Constants.Swerve.*;

/** Add your docs here. */
public class SwerveUtil {
    
    /** 
     * Converts native encoder velocity to meters per second
     * @param encoderVelocity The velocity of the encoder to convert
     * @param gearRatio The current gear ratio that the module is in
     * @return The speed of the module in meters per second
     */
    public static double nativeToMetersPerSecond(double encoderVelocity, double gearRatio) {
        return encoderVelocity * 10 * getMetersPerCount(gearRatio);
    }

    /** 
     * @param encoderCounts
     * @return double
     */
    public static double nativeToRadians(double encoderCounts) {
        return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
    }

    /**
     * Converts meters per second to native encoder counts
     * @param metersPerSecond The speed in meters per second to be converted
     * @param gearRatio The current gear ratio that the module is in
     * @return The equivalent speed motor encoder velocity in counts per 100 ms
     */
    public static double metersPerSecondToNative(double metersPerSecond, double gearRatio) {
        return metersPerSecond / getMetersPerCount(gearRatio) / 10;
    }

    /**
     * Converts radians to native encoder ticks
     * @param radians Radians to convert
     * @return Radians converted to absolute encoder ticks
     */
    public static double radiansToNative(double radians) {
        return radians / (2 * Math.PI) * ANGLE_ENCODER_CPR;
    }

    /**
     * Gets the meters per encoder tick 
     * @param gearRatio Current gear ratio
     * @return The meters per count 
     */
    public static double getMetersPerCount(double gearRatio) {
        return WHEEL_CIRCUMFERENCE / gearRatio / TALON_ENCODER_CPR;
    }
}
