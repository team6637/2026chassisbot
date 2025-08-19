package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String logPath;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    public SwerveModule(String logPath, int moduleNumber, SwerveModuleConstants constants) {
        this.logPath = logPath;
        io = RobotBase.isSimulation()
            ? new SwerveModuleIOSim(constants)   // sim class
            : new SwerveModuleIOReal(moduleNumber, constants); // hardware class
    }

    public void setDesiredState(SwerveModuleState desired) {
        io.setDesiredState(desired);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSecond,
            Rotation2d.fromRotations(inputs.turnAngleRotations)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            Rotation2d.fromRotations(inputs.turnAngleRotations)
        );
    }

    /** Call once per loop from SwerveDrive.periodic() */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);   // AdvantageKit log
    }
}
