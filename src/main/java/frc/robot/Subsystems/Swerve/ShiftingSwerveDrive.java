// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Gyro.IMU;

import static frc.robot.Constants.Swerve.*;

import java.util.concurrent.atomic.AtomicInteger;

public class ShiftingSwerveDrive extends SubsystemBase implements SwerveDrive {

  private ShiftingSwerveModule[] mSwerveModules; 
  private GearShifter mShifter;
  private IMU mGyro;

  private AtomicInteger mCurrentGear;

  private SwerveDriveKinematics mDriveKinematics;
  private SwerveDriveOdometry mDriveOdometry;

  private boolean mFieldCentricActive;
  
  /** Creates a new SwerveDrive. */
  public ShiftingSwerveDrive(GearShifter shifter, IMU gyro) {
    mShifter = shifter;
    mGyro = gyro;

    mCurrentGear = new AtomicInteger(mShifter.getGear());
    
    mSwerveModules = new ShiftingSwerveModule[MODULE_NUM];
    for (int i = 0; i < MODULE_NUM; i++) {
      mSwerveModules[i] = new ShiftingSwerveModule(
        new WPI_TalonFX(SPEED_MOTOR_PORTS[i]),
        new CANSparkMax(ANGLE_MOTOR_PORTS[i], ANGLE_MOTOR_TYPE),
        new AnalogInput(ANGLE_ENCODER_PORT[i]),
        mCurrentGear, 
        SPEED_MOTOR_INVERT[i],
        ANGLE_MOTOR_INVERT[i],
        ANGLE_ENCODER_OFFSETS[i],
        MODULE_GEAR_RATIOS[i]
      );
    }

    mDriveKinematics = new SwerveDriveKinematics(
      BACK_RIGHT_MODULE_POSITION,
      FRONT_RIGHT_MODULE_POSITION,
      FRONT_LEFT_MODULE_POSITION,
      BACK_LEFT_MODULE_POSITION
    );
    mDriveOdometry = new SwerveDriveOdometry(
      mDriveKinematics,
      mGyro.getGyroRotation2d(),
      getModulePositions()
    );
  }
  
  /** 
   * Drives the drivetrain with a standard point of rotation
   * @param fwd Forward velocity
   * @param str Left velocity
   * @param rot Angular velocity
   * @param currentAngle current angle of the robot
   */
  public void drive(double fwd, double str, double rot, Rotation2d currentAngle) {
    drive(fwd, str, rot, currentAngle, new Translation2d());
  }
  
  /** 
   * Drive the drivetrain with a specified point of rotation
   * @param fwd Forward velocity
   * @param str Left velocity
   * @param rot Angular velocity
   * @param currentAngle Current angle of the robot
   * @param pointOfRotation Point the robot will rotate around 
   */
  public void drive(double fwd, double str, double rot, Rotation2d currentAngle, Translation2d pointOfRotation) {
    ChassisSpeeds speeds = mFieldCentricActive == true ?
      ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, currentAngle) : 
      new ChassisSpeeds(fwd, str, rot);
    SwerveModuleState[] moduleStates = mDriveKinematics.toSwerveModuleStates(speeds, pointOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_WHEEL_SPEED);
    drive(moduleStates);
  }

  /** 
   * Drives the drivetrain with a map of SwerveModuleStates 
   * @param moduleStates map of the module states, the key corresponds to the key of the swerve module
   */
  public void drive(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < MODULE_NUM; i++) {
      mSwerveModules[i].drive(moduleStates[i]);
    }
  }
  
  /** 
   * Shifts the gear of the drivetrain
   * @param gear The gear of the robot, 0 is low 1 is high 
   */
  public void shift(int gear) {
    gear = MathUtil.clamp(gear, 0, 1); // Already clamps in DoubleSolenoidSwerveShifter however I do not care
    mCurrentGear.set(gear);
    mShifter.shift(gear);
  }
  
  /** 
   * Gets the module positions from the modules
   * @return An array of the position of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[MODULE_NUM];
    for (int i = 0; i < MODULE_NUM; i++) {
      modulePositions[i] = mSwerveModules[i].getMeasuredPosition();
    }
    return modulePositions;
  }
  
  /** 
   * Updates the pose of the robot using module positions and angle
   * @param modulePositions Positions of the module
   * @param angle Angle of the pose
   */
  public void updatePose(SwerveModulePosition[] modulePositions, Rotation2d angle) {
    mDriveOdometry.update(angle, modulePositions);
  }

  /** 
   * Updates the pose of the robot using a PathPlannerState
   * @param state State of the robot according to PathPlanner
   */
  public void updatePose(PathPlannerState state) {
    mGyro.setGyroAngle(state.holonomicRotation.getRadians());
    Rotation2d rotation = new Rotation2d(state.holonomicRotation.getRadians());
    Pose2d pose = new Pose2d(
        state.poseMeters.getX(), 
        state.poseMeters.getY(), 
        rotation
    );
    mDriveOdometry.resetPosition(
      rotation,
      getModulePositions(),
      pose
    );
  }

  /** 
   * Sets the status of field centric
   * @param fieldCentricActive The desired status of field centric
   */
  public void setFieldCentricActive(boolean fieldCentricActive) {
    mFieldCentricActive = fieldCentricActive;
  }
  
  /** 
   * Gets the status of field centric
   * @return The status of field centric 
   */
  public boolean getFieldCentricActive() {
    return mFieldCentricActive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose(getModulePositions(), mGyro.getGyroRotation2d());
  }
}