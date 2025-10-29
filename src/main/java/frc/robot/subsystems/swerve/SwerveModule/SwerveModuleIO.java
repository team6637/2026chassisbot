package frc.robot.subsystems.swerve.SwerveModule;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    void updateInputs(SwerveModuleIOInputs inputs);
    void setDesiredState(SwerveModuleState state);

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSecond = 0.0;
        public double turnAngleRotations = 0.0;
        public double absoluteAngleDegrees = 0.0;
        public double absoluteAdjustedAngleDegrees = 0.0;
        public double relativeAngleDegrees = 0.0;
        public double desiredMetersPerSecond = 0.0;
    }
}