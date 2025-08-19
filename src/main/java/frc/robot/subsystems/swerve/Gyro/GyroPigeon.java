package frc.robot.subsystems.swerve.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroPigeon implements GyroIO {
    private final Pigeon2 gyro;
    
    public GyroPigeon() {
        this.gyro = new Pigeon2(Constants.Drivetrain.GYRO_ID, Constants.Drivetrain.GYRO_CAN_BUS_NAME);
        configureGyro();
    }

    public void configureGyro() {
        
    }

    @Override
    public void updateInputs(GyroIOInputs inputs, double rotationRate) {
        inputs.yaw =  getYaw();
        inputs.yawDegrees = getYaw().getDegrees();
    }

    @Override
    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    @Override
    public void zeroYaw() {
        gyro.reset();
    }
}
