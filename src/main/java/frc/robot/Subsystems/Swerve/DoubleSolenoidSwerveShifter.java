// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static frc.robot.Constants.PNEUMATICS_MODULE_TYPE;
import static frc.robot.Constants.Swerve.*;

public class DoubleSolenoidSwerveShifter implements GearShifter {

  private final DoubleSolenoid mShifter;

  /** Creates a new DoubleSolenoidSwerveShifter. */
  public DoubleSolenoidSwerveShifter(DoubleSolenoid shifter) {
    this.mShifter = new DoubleSolenoid(
      PNEUMATICS_MODULE_TYPE, 
      REVERSE_CHANNEL, 
      FORWARD_CHANNEL
    );
  }

  public void shift(int gear) {
    gear = MathUtil.clamp(gear, 0, 1);
    mShifter.set(gear == 0 ? LO_GEAR_VALUE : HI_GEAR_VALUE);
  }

  /*
   * 1 is high gear and 0 is low gear
   */
  public int getGear() {
    return this.mShifter.get() == HI_GEAR_VALUE ? 1 : 0;
  }

}