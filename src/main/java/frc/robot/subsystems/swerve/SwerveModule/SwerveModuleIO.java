package frc.robot.subsystems.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    void updateInputs(SwerveModuleIOInputs inputs);
    void setDesiredState(SwerveModuleState state);
}