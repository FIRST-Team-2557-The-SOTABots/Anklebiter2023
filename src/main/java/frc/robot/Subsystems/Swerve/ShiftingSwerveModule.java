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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Gyro.NavX;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Subsystems.Swerve.SwerveUtil.*;

import java.util.concurrent.atomic.AtomicInteger;

public class ShiftingSwerveModule extends SubsystemBase implements SwerveModule {

  private WPI_TalonFX mSpeedMotor;
  private CANSparkMax mAngleMotor;
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
      WPI_TalonFX speedMotor, 
      CANSparkMax angleMotor,
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
    return SwerveUtil.nativeToMetersPerSecond(
      mSpeedMotor.getSelectedSensorVelocity(), 
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
