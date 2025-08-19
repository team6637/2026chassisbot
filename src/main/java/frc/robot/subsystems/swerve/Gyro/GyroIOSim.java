package frc.robot.subsystems.swerve.Gyro;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
    private Rotation2d yaw = new Rotation2d();
    double lastInputsUpdateTime = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(GyroIOInputs inputs, double rotationRateRadiansPerSecond) {
        double currentTime = Timer.getFPGATimestamp();  
        double deltaTimeSecond = currentTime - this.lastInputsUpdateTime;
        this.lastInputsUpdateTime = currentTime;

        inputs.yaw = Rotation2d.fromRadians(((rotationRateRadiansPerSecond * deltaTimeSecond) + inputs.yaw.getRadians()));
        inputs.yawDegrees =  MathUtil.inputModulus(yaw.getDegrees(), -180.0, 180.0);

        yaw = inputs.yaw;
    }

    @Override
    public Rotation2d getYaw() {
        return yaw;
    }

    @Override
    public void zeroYaw() {
        yaw = new Rotation2d();
    }
}
