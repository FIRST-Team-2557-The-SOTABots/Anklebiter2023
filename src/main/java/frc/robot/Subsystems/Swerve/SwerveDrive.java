// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveDrive {
    public void drive(double fwd, double lft, double rot, Rotation2d currentAngle);
    public void drive(double fwd, double lft, double rot, Rotation2d currentAngle, Translation2d pointOfRotation);
    public void drive(SwerveModuleState[] moduleStates);
    public void setFieldCentricActive(boolean fieldCentricActive);
    public boolean getFieldCentricActive();
}
