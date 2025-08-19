package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleConstants;

public final class Constants {

    public static final class Drivetrain {
        // Physical robot dimensions (meters)
        public static final double TRACK_WIDTH = 0.576; // 22.68 in
        public static final double WHEEL_BASE = 0.576;  // 22.68 in

        // Gyro
        public static final int GYRO_ID = 9;
        public static final String GYRO_CAN_BUS_NAME = "Gary";

        public static final double MAXIMUM_CHASSIS_VELOCITY = 4.0; // m/s
        public static final double MAXIMUM_CHASSIS_ANGULAR_VELOCITY = 10.0; // rad/s
    }

    public static final class Controls {
        // Joystick deadband for angle-only control (e.g. rotation stick ring)
        public static final double ANGLE_JOYSTICK_DEADBAND = 0.5;
        public static final double Y_DEADBAND = 0.2;
    
        // PID values for heading hold (used for snapping or rotating to angle)
        public static final double HEADING_kP = 3.0;
        public static final double HEADING_kI = 0.0;
        public static final double HEADING_kD = 0.01;
    }

    public static final class SwerveConfig {
        // Gearing
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double ANGLE_GEAR_RATIO = 21.4285714286;
    
        // used to adjust auto drive distance
        // multiply by wheel diameter to fudge from tuning steps
        public static final double fudge = 1;

        // Wheel diameter in meters (4 inch wheel)
        public static final double WHEEL_DIAMETER_METERS = 0.1016 * fudge; // 4 inches

        public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    
        // Conversion factors
        public static final double NEO_FREE_SPEED = 5676;
        public static final double FREE_SPEED_RPS = NEO_FREE_SPEED / 60;
        public static final double DRIVE_FREE_SPEED_RPS = (FREE_SPEED_RPS * WHEEL_CIRCUMFRENCE_METERS) / DRIVE_GEAR_RATIO;

        public static final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFRENCE_METERS / DRIVE_GEAR_RATIO; // meters/rev
        public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0; // m/s per RPM
    
        public static final double ANGLE_POSITION_CONVERSION = 1 / ANGLE_GEAR_RATIO; // motor revolutions per 1 swerve module rotation
    
        // Current limits
        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 20;
    
        // Ramp rates
        public static final double DRIVE_RAMP = 0.25; // seconds to full throttle
        public static final double ANGLE_RAMP = 0.25;
    
        // PID (from JSON)
        public static final double DRIVE_kP = 0.0020645;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double DRIVE_kF = 0.0;
    
        public static final double ANGLE_kP = 5.0;
        public static final double ANGLE_kI = 0.0;
        public static final double ANGLE_kD = 0.0;
        public static final double ANGLE_kF = 0.0;

    }
  

    public static final class SwerveModules {
        public static final SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
            11,
            12,
            13,
            "Gary",
            true,
            true,
            Rotation2d.fromDegrees(204.79), // degrees
            new Translation2d(Drivetrain.WHEEL_BASE / 2.0, Drivetrain.TRACK_WIDTH / 2.0)
        );

        public static final SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(
            21, 
            22, 
            23, 
            "Gary",
            true, true,
            Rotation2d.fromDegrees(191.34),
            new Translation2d(Drivetrain.WHEEL_BASE / 2.0, -Drivetrain.TRACK_WIDTH / 2.0)
        );

        public static final SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(
            31, 
            32, 
            33,
            "Gary",
            true, true,
            Rotation2d.fromDegrees(50.19),
            new Translation2d(-Drivetrain.WHEEL_BASE / 2.0, Drivetrain.TRACK_WIDTH / 2.0)
        );

        public static final SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(
            37, 
            38, 
            39,
            "Gary",
            true, true,
            Rotation2d.fromDegrees(352.71),
            new Translation2d(-Drivetrain.WHEEL_BASE / 2.0, -Drivetrain.TRACK_WIDTH / 2.0)
        );
    }
}
