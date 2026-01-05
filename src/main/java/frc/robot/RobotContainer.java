// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import static frc.robot.subsystems.vision.VisionConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {

    // Subsystems
    private final Vision vision;
    private final SwerveDrive swerveDrive = new SwerveDrive();

    // Driver Joystick
    private final Joystick driverJoystick = new Joystick(0);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        if(!RobotBase.isSimulation()) {
            vision =
            new Vision(
                swerveDrive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1)
            );
        } else {
            vision =
            new Vision(
                swerveDrive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, swerveDrive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, swerveDrive::getPose));
        }

        autoChooser = AutoBuilder.buildAutoChooser();

        configureBindings();

        // Default driving command (joystick)
        swerveDrive.setDefaultCommand(
            new TeleopDriveCommand(
                swerveDrive,
                ()-> -MathUtil.applyDeadband(driverJoystick.getY(), Constants.Controls.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getX(), Constants.Controls.Y_DEADBAND),
                ()-> -MathUtil.applyDeadband(driverJoystick.getTwist(), Constants.Controls.ANGLE_JOYSTICK_DEADBAND)
            )
        );
        
        printDebugValues();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }


    private void configureBindings() {
        //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    }

    private void printDebugValues() {
        // add smart dashboard debug calls here instead of in subsystems
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return new TestBangBang(swerveDrive);
    }
}
