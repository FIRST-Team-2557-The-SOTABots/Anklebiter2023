// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Subsystems.Gyro.NavX;

import static frc.robot.Constants.Swerve.*;

/** Add your docs here. */
public class RobotState {

    private static RobotState mInstance = null;
    
    /*
     *A representation of the gear so that both the swerve module and swerve drive
     * can tell which gear the drive train is in
     * TODO: I don't know how much I like this solution
     */
    private int mGear; // 0 is low hear and 1 is high gear

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState() {}

    /*
     * Sets the gear
     */
    public void setGear(int gear) {
        mGear = gear;
    }

    /*
     * Gets the gear
     */
    public int getGear() {
        return mGear;
    }
    
}
