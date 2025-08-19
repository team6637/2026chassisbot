package frc.robot.subsystems.swerve.Gyro;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    Rotation2d getYaw();
    void zeroYaw();

    @AutoLog
    public static class GyroIOInputs  {
        public Rotation2d yaw = new Rotation2d();
        public double yawDegrees = 0.0;
    }

    // sim uses rotationRate to keep track of bot rotation
    // rotationRate is unused by real gyro
    public default void updateInputs(GyroIOInputs inputs,  double rotationRate) {}
}