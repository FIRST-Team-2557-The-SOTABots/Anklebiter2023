// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Gyro.NavX;

import static frc.robot.Constants.Swerve.*;

import java.util.concurrent.atomic.AtomicInteger;

public class ShiftingSwerveModule extends SubsystemBase implements SwerveModule {

  private MotorController mSpeedMotor;
  private MotorController mAngleMotor;
  private AnalogInput mAngleEncoder;
  private double kAngleEncoderOffset;

  private AtomicInteger mCurrentGear;

  private PIDController mSpeedPID;
  private ProfiledPIDController mAnglePID;

  // This is a solution for multiple gear ratios per module will get rid of in later version
  // 0 is low 1 is high
  private double[] mGearRatios;

  

  /** Creates a new SwerveModule. */
  public ShiftingSwerveModule(
    //changed to motorcontroller so it uses the interface this still works with the talons
      MotorController speedMotor, 
      MotorController angleMotor,
      AnalogInput angleEncoder,
      AtomicInteger currentGear, 
      boolean speedInverted,
      boolean angleInverted,
      double angleEncoderOffset,
      double[] gearRatios) {
    this.mSpeedMotor = speedMotor;
    this.mAngleMotor = angleMotor;
    this.mAngleEncoder = angleEncoder;

    speedMotor.setInverted(speedInverted);
    angleMotor.setInverted(angleInverted);
    kAngleEncoderOffset = angleEncoderOffset;
    
    mCurrentGear = currentGear;
    mGearRatios = gearRatios;

    mSpeedPID = new PIDController(
      SPEED_PID_KP, 
      SPEED_PID_KI, 
      SPEED_PID_KD
    );
    mAnglePID = new ProfiledPIDController(
      ANGLE_PID_KP, 
      ANGLE_PID_KI, 
      ANGLE_PID_KD, 
      new TrapezoidProfile.Constraints(
        ANGLE_PID_MAX_VELOCITY,
        ANGLE_PID_MAX_ACCELERATION
      )
    );
    mAnglePID.enableContinuousInput(0, ANGLE_ENCODER_CPR);
    mAnglePID.setTolerance(ANGLE_PID_TOLERANCE);
  }
  
  /** 
   * Drives the swerve module with a SwerveModuleState
   * @param state Desired swerve module state
   */
  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(nativeToRadians(getAngle())));

    double angleSetpointNative = radiansToNative(state.angle.getRadians());
    double anglePIDOutput = mAnglePID.calculate(getAngle(), angleSetpointNative);
    
    if (state.speedMetersPerSecond == 0.0)
      mAngleMotor.setVoltage(0.0);
    else 
      mAngleMotor.setVoltage(anglePIDOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, mGearRatios[mCurrentGear.get()]);
    double speedPIDOutput = mSpeedPID.calculate(getSpeed(), speedSetpointNative);
    
    mSpeedMotor.setVoltage(speedPIDOutput);
  }
  
  /** 
   * Gets the swerve module state of the module
   * @return The SwerveModuleState of the module
   */
  public SwerveModuleState getMeasuredState() {
    SwerveModuleState state = new SwerveModuleState(
      getSpeed(), 
      //TODO: check to see if the navx is the best way of getting the gyroangle
      new Rotation2d(nativeToRadians(NavX.getInstance().getGyroAngle()))
    );
    if (state.speedMetersPerSecond < 0.0) {
      state.speedMetersPerSecond *= -1;
      state.angle = state.angle.plus(new Rotation2d(Math.PI));
    }
    return state;
  }
  
  /** 
   * Gets the swerve module position of the module
   * @return The SwerveModulePosition of the module
   */
  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(
      WHEEL_CIRCUMFERENCE, 
      new Rotation2d(getAngle())
    );
  }

  /**
   * Gets the speed of the module in meters per second
   * @return The speed of the module in meters per second
   */
  public double getSpeed() {
    return nativeToMetersPerSecond(
      mSpeedMotor.get(), //TODO: check to see if this should be used for all motor controllers
      mGearRatios[mCurrentGear.get()]
    );
  }

  /** 
   * Gets the angle of the module in absolute encoder ticks
   * @return The angle of the module in absolute encoder ticks
   */
  public double getAngle() {
    return -1 * MathUtil.inputModulus(mAngleEncoder.getAverageVoltage() - kAngleEncoderOffset, 0, ANGLE_ENCODER_CPR) + ANGLE_ENCODER_CPR;
  }

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}