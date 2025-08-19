package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Gyro.GyroIO;
import frc.robot.subsystems.swerve.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.Gyro.GyroIOSim;
import frc.robot.subsystems.swerve.Gyro.GyroPigeon;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModule.SwerveModuleIOInputs;


public class SwerveDrive extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();


    // Modules
    private final SwerveModule[] modules = {
        new SwerveModule("Swerve/FL",0, Constants.SwerveModules.FRONT_LEFT),
        new SwerveModule("Swerve/FR", 1, Constants.SwerveModules.FRONT_RIGHT),
        new SwerveModule("Swerve/BL", 2, Constants.SwerveModules.BACK_LEFT),
        new SwerveModule("Swerve/BR", 3, Constants.SwerveModules.BACK_RIGHT)
    };

    private final SwerveModuleIOInputs[] moduleInputs = {
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs(),
        new SwerveModuleIOInputs()
    };

    // Kinematics and odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        Constants.SwerveModules.FRONT_LEFT.modulePosition,
        Constants.SwerveModules.FRONT_RIGHT.modulePosition,
        Constants.SwerveModules.BACK_LEFT.modulePosition,
        Constants.SwerveModules.BACK_RIGHT.modulePosition
    );

    private final SwerveDriveOdometry odometry; 

    public SwerveDrive() {

        // Gyro
        if(RobotBase.isSimulation()) {
            gyro = new GyroIOSim();
        } else {
            gyro = new GyroPigeon();
        }
        gyro.zeroYaw();

        odometry = new SwerveDriveOdometry(
            kinematics,
            getHeading(),
            getModulePositions()
        );
    }

    public Rotation2d getHeading() {
        return gyro.getYaw();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Resets odometry to a known pose */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(this.getModuleStates());
    }

    /** Returns current state of all modules */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns current positions of all modules */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // Drive Field Relative (x, y z)
    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity) {
        this.drive(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity, true);
    }

    // Drive (x, y, z, isFieldRelative)
    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity, boolean isFieldRelative) {
        if(isFieldRelative) {
            this.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity, getHeading());
        } else {
            this.desiredChassisSpeeds = new ChassisSpeeds(desiredXVelocity, desiredYVelocity, desiredRotationalVelocity);
        }            
    }

    // send the values to the pods
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }

        // Log Desired Angle and Speed of Modules for Advantage Scope swerve pod tuning
        Logger.recordOutput("Swerve/MyDesiredStates", desiredStates);
    }

    @Override
    public void periodic() {
        if(desiredChassisSpeeds != null) {
            SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
            setModuleStates(desiredStates);
        }
        log();
        desiredChassisSpeeds = null;

        // Update odometry
        odometry.update(getHeading(), getModulePositions());
    }

    public void log() {
        // Update inputs for logging
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs();
            Logger.processInputs("Swerve/Module" + i, moduleInputs[i]);
        }

        gyro.updateInputs(gyroInputs, getChassisSpeeds().omegaRadiansPerSecond);

        Logger.processInputs("Swerve/Gyro", gyroInputs);
        Logger.recordOutput("Swerve/Pose", getPose());

        // Log Swerve Module States for pod tuning
        Logger.recordOutput("Swerve/MyStates", getModuleStates());
    }

}
