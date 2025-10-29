package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final String logPath;
    private final SwerveModuleIO io;

    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    public SwerveModule(String logPath, int moduleNumber, SwerveModuleConstants constants) {
        this.logPath = logPath;

        if(RobotBase.isSimulation()) {
            io = new SwerveModuleIOSim(constants);
        } else {
            io = new SwerveModuleIOReal(moduleNumber, constants);
        }
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

    public SwerveModuleIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logPath, inputs);
    }
}
