// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Subsystems.Gyro.NavX;

import static frc.robot.Constants.Swerve.*;

public class ShiftingSwerveDrive extends SubsystemBase implements SwerveDrive {

  private static ShiftingSwerveDrive mInstance = null;

  private ShiftingSwerveModule[] mSwerveModules; 
  private DoubleSolenoidSwerveShifter mShifter;
  private NavX mNavX;

  private SwerveDriveKinematics mDriveKinematics;
  private SwerveDriveOdometry mDriveOdometry;

  private boolean mFieldCentricActive;
  
  /** 
   * Gets the instance of the swerve drive
   * @return SwerveDrive
   */
  public ShiftingSwerveDrive getInstance() {
    if (mInstance == null) {
      mInstance = new ShiftingSwerveDrive(NavX.getInstance());
    }
    return mInstance;
  }
  
  /** Creates a new SwerveDrive. */
  private ShiftingSwerveDrive(NavX navX) {
    // mSwerveModules = new ShiftingSwerve 
    // Note: this module is one of the new modules

    mShifter = DoubleSolenoidSwerveShifter.getInstance();
    mNavX = navX;

    mDriveKinematics = new SwerveDriveKinematics(
      BACK_RIGHT_MODULE_POSITION,
      FRONT_RIGHT_MODULE_POSITION,
      FRONT_LEFT_MODULE_POSITION,
      BACK_LEFT_MODULE_POSITION
   );
    mDriveOdometry = new SwerveDriveOdometry(
      mDriveKinematics,
      new Rotation2d(NavX.getInstance().getGyroAngle()),
      getModulePositions()
    );
  }
  
  /** 
   * Drives the drivetrain with a standard point of rotation
   * @param fwd Forward velocity
   * @param lft Left velocity
   * @param rot Angular velocity
   * @param currentAngle current angle of the robot
   */
  public void drive(double fwd, double lft, double rot, Rotation2d currentAngle) {
    drive(fwd, lft, rot, currentAngle, new Translation2d());
  }
  
  /** 
   * Drive the drivetrain with a specified point of rotation
   * @param fwd Forward velocity
   * @param lft Left velocity
   * @param rot Angular velocity
   * @param currentAngle Current angle of the robot
   * @param pointOfRotation Point the robot will rotate around 
   */
  public void drive(double fwd, double lft, double rot, Rotation2d currentAngle, Translation2d pointOfRotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd, lft, rot, currentAngle);
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
    RobotState.getInstance().setGear(gear);
    mShifter.shift(gear);
  }

  public void setFieldCentricActive(boolean fieldCentricActive) {
    mFieldCentricActive = fieldCentricActive;
  }

  public boolean getFieldCentricActive() {
    return mFieldCentricActive;
  }
  
  /** 
   * Gets the module positions from the modules
   * @return SwerveModulePosition[]
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
    mNavX.setGyroAngle(state.holonomicRotation.getRadians());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose(getModulePositions(), mNavX.getGyroRotation2d());
  }
}
