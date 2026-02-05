// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Class
public class PhotonSubsystem extends SubsystemBase {

    private PhotonCamera camera1; //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;

    private PhotonSubsystem() { //private? or public?
        camera1 = new PhotonCamera(""); //make sure this name matches the camera name in photonvision interface
    }

    @Override
    public void periodic() {
        result = camera1.getLatestResult();

        if (result.hasTargets()) {
            target = result.getBestTarget();
        }

        else {
            target = null;
        }
    }

    // Returns true if the camera detects an AprilTag
    public boolean hasTarget() {
        return target != null;
    }

    // Returns yaw (left/right angle), or 0 if the target is centered
    public double getYaw() {
        if (!hasTarget()) return 0;
        return target.getYaw();
    }
    //Returns Skew(angle of the target), or 0 if no target
    public double getSkew() {
        if (!hasTarget()) return 0;
        return target.getSkew();
    }
    // Returns pitch (up/down angle), or 0 if no target
    public double getPitch() {
        if (!hasTarget()) return 0;
        return target.getPitch();
    }

    // Returns AprilTag ID, or -1 if no target
    public int getTagID() {
        if (!hasTarget()) return -1;
        return target.getFiducialId();
    }

}
