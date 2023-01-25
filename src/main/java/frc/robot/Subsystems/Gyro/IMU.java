// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IMU {
    public double getGyroAngle();
    public Rotation2d getGyroRotation2d();
    public void setGyroAngle(double radians);
    public void resetGyro();
}
