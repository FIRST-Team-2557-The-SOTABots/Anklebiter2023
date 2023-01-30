// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class Constants {

    public static final double METERS_PER_INCH = 0.0254; // Thanks Hayden

    //TODO: not too sure where to put this
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

    public static final SerialPort.Port NAVX_PORT = SerialPort.Port.kMXP;

    public static final class Controller {
        public static final class Driver {
            public static final int PORT = 0;
            public static final double STICK_DEADBAND = 0.1;
        }

        public static final class Manipulator {
            public static final int PORT = 1;
        }
    }

    public static final class Swerve {

        public static final int MODULE_NUM = 4;

        // Order BR FR FL BL TODO: double check this
        public static final int[] SPEED_MOTOR_PORTS = {8, 1, 4, 5};
        public static final int[] ANGLE_MOTOR_PORTS = {7, 2, 3, 6};
        public static final int[] ANGLE_ENCODER_PORT = {0, 1, 2, 3};

        public static final boolean[] SPEED_MOTOR_INVERT = {true, false, false, true};
        public static final boolean[] ANGLE_MOTOR_INVERT = {false, true, false, false};

        public static final MotorType ANGLE_MOTOR_TYPE = MotorType.kBrushless;

        public static final double[] ANGLE_ENCODER_OFFSETS = {4.044189039, 0.977783103, 1.697997873, 3.328857081}; 

        public static final int FORWARD_CHANNEL = 0;
        public static final int REVERSE_CHANNEL = 1;

        public static final Value HI_GEAR_VALUE = Value.kForward; 
        public static final Value LO_GEAR_VALUE = Value.kReverse; 
        public static final int HI_GEAR_INT = 1;
        public static final int LO_GEAR_INT = 0;        

        public static final double WHEEL_BASE = 23.111 * METERS_PER_INCH; // TODO: talk to jay about these numbers
        public static final double TRACK_WIDTH = 23.111 * METERS_PER_INCH; // TODO: good?
        public static final double WHEEL_DIAMETER = 4.0 * METERS_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // TODO: im using this as wheel distance double check if this is real

        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);

        public static final double TALON_ENCODER_CPR = 2048;
        public static final double ANGLE_ENCODER_CPR = 5.0; // TODO: test this

        public static final double[] OLD_DRIVE_GEAR_RATIOS = {13.68 / 1, 6.50 / 1};
        public static final double[] NEW_DRIVE_GEAR_RATIOS = {10.40 / 1, 5.07 / 1};
        public static final double[][] MODULE_GEAR_RATIOS = {OLD_DRIVE_GEAR_RATIOS, NEW_DRIVE_GEAR_RATIOS, OLD_DRIVE_GEAR_RATIOS, OLD_DRIVE_GEAR_RATIOS};

        // In meters per second
        public static final double MAX_WHEEL_SPEED = 5.2;

        public static final double MAX_ANGLULAR_SPEED = 15; // TODO: Unnoficial number

        // TODO: Tune PID
        public static final double SPEED_PID_KP = 0.0001; 
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;
        public static final double SPEED_PID_TOLERANCE = 0.05;

        public static final double ANGLE_PID_KP = 2.0;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double[] ANGLE_PID_VALUES = {ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD};
        public static final double ANGLE_PID_TOLERANCE = 0.0;
        public static final double ANGLE_PID_MAX_ACCELERATION = 70.0;
        public static final double ANGLE_PID_MAX_VELOCITY = ANGLE_PID_MAX_ACCELERATION * Math.sqrt((ANGLE_ENCODER_CPR / 4) / ANGLE_PID_MAX_ACCELERATION);
        
        public static final double NEW_ANGLE_PID_KP = 1;
        public static final double NEW_ANGLE_PID_KI = 0.0;
        public static final double NEW_ANGLE_PID_KD = 0.0;
        public static final double[] NEW_ANGLE_PID_VALUES = {NEW_ANGLE_PID_KP, NEW_ANGLE_PID_KI, NEW_ANGLE_PID_KD};

    }
}