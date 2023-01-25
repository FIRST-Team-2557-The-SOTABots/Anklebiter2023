// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModule {
    public void drive(SwerveModuleState state);
    public double getSpeed();
    public double getAngle();
    public SwerveModuleState getMeasuredState();
    public SwerveModulePosition getMeasuredPosition();

}