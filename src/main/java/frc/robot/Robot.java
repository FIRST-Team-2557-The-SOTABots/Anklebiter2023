// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Gyro.NavX;
import frc.robot.Subsystems.Swerve.DoubleSolenoidSwerveShifter;
import frc.robot.Subsystems.Swerve.ShiftingSwerveDrive;

import static frc.robot.Constants.*;

public class Robot extends TimedRobot {
  // private Command mAutonomousCommand;

  // private RobotContainer m_robotContainer;

  private CommandXboxController mDriveStick = new CommandXboxController(Controller.Driver.PORT);
  // private XboxController mDriverStick = new XboxController(Controller.Driver.PORT);

  private NavX mNavX = NavX.getInstance();
  private DoubleSolenoidSwerveShifter mShifter = new DoubleSolenoidSwerveShifter(
    new DoubleSolenoid(PNEUMATICS_MODULE_TYPE, Swerve.FORWARD_CHANNEL, Swerve.REVERSE_CHANNEL)  
);
  private ShiftingSwerveDrive mSwerveDrive = new ShiftingSwerveDrive(
    mShifter, 
    mNavX
  );

  private void configureButtonBindings() {
    mDriveStick.a().onTrue(
      new InstantCommand(() -> {
        // Inverts field centric
        mSwerveDrive.setFieldCentricActive(!mSwerveDrive.getFieldCentricActive());
      })
    );
  }

  private void configureDefaultCommands() {
    mSwerveDrive.setDefaultCommand( 
      new RunCommand(() -> {
        // Process gear shift inputs
        if (mDriveStick.getRightTriggerAxis() > 0.0) mSwerveDrive.shift(Swerve.LO_GEAR_INT);
        else mSwerveDrive.shift(Swerve.HI_GEAR_INT);

        // Process stick inputs
        double fwd = mDriveStick.getLeftY();
        double str = mDriveStick.getLeftX();
        double rot = mDriveStick.getRightX();

        // Reverses the input, squares the input (uses signum to preserve the sign), and scale to max wheel speed
        fwd = -Math.signum(fwd) * fwd * fwd * Swerve.MAX_WHEEL_SPEED; 
        str = -Math.signum(str) * str * str * Swerve.MAX_WHEEL_SPEED;
        rot = -Math.signum(rot) * rot * rot * Swerve.MAX_ANGLULAR_SPEED;

        SmartDashboard.putNumber("fwd", fwd);
        SmartDashboard.putNumber("str", str);        
        SmartDashboard.putNumber("rot", rot);

        // Passes inputs into drivetrain
        mSwerveDrive.drive(fwd, str, rot, mNavX.getGyroRotation2d());

      },
        mSwerveDrive
      )
    );
  }

  @Override
  public void robotInit() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Field Centric Active", mSwerveDrive.getFieldCentricActive());
    SmartDashboard.putNumber("Right Trigger", mDriveStick.getRightTriggerAxis());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
