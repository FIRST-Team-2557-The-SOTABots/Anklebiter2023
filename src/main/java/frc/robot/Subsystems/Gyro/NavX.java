// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX implements IMU {

  private static NavX mInstance = null;

  private final AHRS mNavX;

  public static NavX getInstance() {
    if (mInstance == null) {
      mInstance = new NavX();
    }
    return mInstance;
  }

  /** Creates a new NavX. */
  public NavX() {
    mNavX = new AHRS(SerialPort.Port.kMXP);
  }

  /*
   * Gets the NavX's angle in radians
   */
  public double getGyroAngle() {
    return -Math.toRadians(mNavX.getAngle());
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(getGyroAngle());
  }

  /*
   * Sets the NavX's Angle
   */
  public void setGyroAngle(double radians) {
    mNavX.reset();
    mNavX.setAngleAdjustment(-Math.toDegrees(radians));
  }

  /*
   * Resets the NavX's angle
   */
  public void resetGyro() {
    mNavX.setAngleAdjustment(0.0);
    mNavX.reset();
  }

}
