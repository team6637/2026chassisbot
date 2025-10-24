// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;


import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
    PhotonCamera frontCamera = new PhotonCamera("frontcamera");
    PhotonCamera backCamera = new PhotonCamera("backcamera");

    int pipeline;
    boolean frontHasTargets, backHasTargets;

    List<PhotonTrackedTarget> frontTargets, backTargets;
    PhotonTrackedTarget frontBestTarget, backBestTarget;


    public VisionSubsystem(int pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        var frontResult = frontCamera.getLatestResult();
        var backResult = backCamera.getLatestResult();
        
        frontHasTargets = frontResult.hasTargets();
        if (frontHasTargets) {
            frontTargets = frontResult.getTargets();
            frontBestTarget = frontResult.getBestTarget();

            /* 
            SmartDashboard.putNumber("frontX", frontBestTarget.getBestCameraToTarget().getX());
            SmartDashboard.putNumber("frontY", frontBestTarget.getBestCameraToTarget().getY());
            SmartDashboard.putNumber("frontZ", frontBestTarget.getBestCameraToTarget().getZ());
            */
        }
        backHasTargets = backResult.hasTargets();
        if (backHasTargets) {
            backTargets = backResult.getTargets();
            backBestTarget = backResult.getBestTarget();

            /*
            SmartDashboard.putNumber("backX", backBestTarget.getBestCameraToTarget().getX());
            SmartDashboard.putNumber("backY", backBestTarget.getBestCameraToTarget().getY());
            SmartDashboard.putNumber("backZ", backBestTarget.getBestCameraToTarget().getZ());
            */
        }
    }
}
