// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.VisionSubsystem;

public class RobotContainer {

    // Subsystems
    private final SwerveDrive swerveDrive = new SwerveDrive();
    //private final VisionSubsystem vision = new VisionSubsystem(0);

    // Driver Joystick
    private final Joystick driverJoystick = new Joystick(0);

    public RobotContainer() {
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
        return new PathPlannerAuto("Test Auto");
    }
}
