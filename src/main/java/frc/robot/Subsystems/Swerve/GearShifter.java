// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

/** Add your docs here. */
// TODO: Find a place to put this
public interface GearShifter {
    public void shift(int gear);
    public int getGear();
}