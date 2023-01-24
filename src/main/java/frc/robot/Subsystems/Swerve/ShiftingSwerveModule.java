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
import frc.robot.RobotState;
import frc.robot.Subsystems.Gyro.NavX;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Subsystems.Swerve.SwerveUtil.*;

import java.util.concurrent.atomic.AtomicInteger;


public class ShiftingSwerveModule extends SubsystemBase implements SwerveModule {

  private static ShiftingSwerveModule mFrontRightInstance = null;
  private static ShiftingSwerveModule mFrontLeftInstance = null;
  private static ShiftingSwerveModule mBackRightInstance = null;
  private static ShiftingSwerveModule mBackLeftInstance = null;

  private WPI_TalonFX mSpeedMotor;
  private CANSparkMax mAngleMotor;
  private AnalogInput mAngleEncoder;
  private double kAngleEncoderOffset;

  private PIDController mSpeedPID;
  private ProfiledPIDController mAnglePID;

  private AtomicInteger mCurrentGear;

  // This is a solution for multiple gear ratios per module will get rid of in later version
  // 0 is low 1 is high
  private double[] mGearRatios;

  public static ShiftingSwerveModule getBackRightInstance() {
    if (mBackRightInstance == null) {
      mBackRightInstance = new ShiftingSwerveModule(
        new WPI_TalonFX(SPEED_MOTOR_PORTS[0]),
        new CANSparkMax(ANGLE_MOTOR_PORTS[0], ANGLE_MOTOR_TYPE),
        new AnalogInput(ANGLE_ENCODER_PORT[0]),
        SPEED_MOTOR_INVERT[0],
        ANGLE_MOTOR_INVERT[0],
        ANGLE_ENCODER_OFFSETS[0],
        OLD_DRIVE_GEAR_RATIOS
      );
    }
    return mBackRightInstance;
  }

  public static ShiftingSwerveModule getFrontRightInstance() {
    if (mFrontRightInstance == null) {
      mFrontRightInstance = new ShiftingSwerveModule(
        new WPI_TalonFX(SPEED_MOTOR_PORTS[1]),
        new CANSparkMax(ANGLE_MOTOR_PORTS[1], ANGLE_MOTOR_TYPE),
        new AnalogInput(ANGLE_ENCODER_PORT[1]),
        SPEED_MOTOR_INVERT[1],
        ANGLE_MOTOR_INVERT[1],
        ANGLE_ENCODER_OFFSETS[1],
        NEW_DRIVE_GEAR_RATIOS
      );
    }
    return mFrontRightInstance;
  }

  public static ShiftingSwerveModule getFrontLeftInstance() {
    if (mFrontLeftInstance == null) {
      mFrontLeftInstance = new ShiftingSwerveModule(
        new WPI_TalonFX(SPEED_MOTOR_PORTS[2]),
        new CANSparkMax(ANGLE_MOTOR_PORTS[2], ANGLE_MOTOR_TYPE),
        new AnalogInput(ANGLE_ENCODER_PORT[2]),
        SPEED_MOTOR_INVERT[2],
        ANGLE_MOTOR_INVERT[2],
        ANGLE_ENCODER_OFFSETS[2],
        OLD_DRIVE_GEAR_RATIOS
      );
    }
    return mFrontLeftInstance;
  }

  public static ShiftingSwerveModule getBackLeftInstance() {
    if (mBackLeftInstance == null) {
      mBackLeftInstance = new ShiftingSwerveModule(
        new WPI_TalonFX(SPEED_MOTOR_PORTS[3]),
        new CANSparkMax(ANGLE_MOTOR_PORTS[3], ANGLE_MOTOR_TYPE),
        new AnalogInput(ANGLE_ENCODER_PORT[3]),
        SPEED_MOTOR_INVERT[3],
        ANGLE_MOTOR_INVERT[3],
        ANGLE_ENCODER_OFFSETS[3],
        OLD_DRIVE_GEAR_RATIOS
      );
    }
    return mBackLeftInstance;
  }

  /** Creates a new SwerveModule. */
  public ShiftingSwerveModule(
      WPI_TalonFX speedMotor, 
      CANSparkMax angleMotor,
      AnalogInput angleEncoder, 
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

  public void drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(nativeToRadians(getAngle())));

    double angleSetpointNative = radiansToNative(state.angle.getRadians());
    double anglePIDOutput = mAnglePID.calculate(getAngle(), angleSetpointNative);
    
    if (state.speedMetersPerSecond == 0.0)
      mAngleMotor.setVoltage(0.0);
    else 
      mAngleMotor.setVoltage(anglePIDOutput);

    double speedSetpointNative = metersPerSecondToNative(state.speedMetersPerSecond, )

  }

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
  
  public SwerveModulePosition getMeasuredPosition() {
    return new SwerveModulePosition(
      WHEEL_CIRCUMFERENCE, 
      new Rotation2d(getAngle())
    );
  }

  /**
   * 
   * @return the speed of the module in meters per second
   */
  public double getSpeed() {
    return SwerveUtil.nativeToMetersPerSecond(
      mSpeedMotor.getSelectedSensorVelocity(), 
      mGearRatios[RobotState.getInstance().getGear()]
    );
  }

  public double getAngle() {
    return -1 * MathUtil.inputModulus(mAngleEncoder.getAverageVoltage() - kAngleEncoderOffset, 0, ANGLE_ENCODER_CPR) + ANGLE_ENCODER_CPR;
  }

  public int getGear() {
    return mGear
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
