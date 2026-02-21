// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
//import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.SwerveSubsystem; // For gyro, gives error because it's on a different branch

// Class
public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1; //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    //private final SwerveSubsystem m_SwerveSubsystem;
                            
    public PhotonSubsystem() { //private? or public?
        camera1 = new PhotonCamera("Arducam"); //make sure this name matches the camera name in photonvision interface
        //m_SwerveSubsystem = swerveSubsystem;
    }

    @Override
    public void periodic() {
        result = camera1.getLatestResult();

        if (result.hasTargets()) {
            target = result.getBestTarget();
            System.out.println(target);
        }

        else {
            System.out.println("No target found");
            target = null;

        }
        System.out.println("Pitch:" + getPitch());
        System.out.println("Yaw:" + getYaw());
        System.out.println("Skew" + getSkew());
        System.out.println("Apriltag: " + getTagID());
    }

    // Returns true if the camera detects an AprilTag
    public boolean hasTarget() {
        return result != null && result.hasTargets();
    }

    // Returns yaw (left/right angle), or 0 if the target is centered
    public double getYaw() {
        if (hasTarget()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }
    //Returns Skew (angle of the target), or 0 if no target
   public double getSkew() {
        if (hasTarget()) {
            return result.getBestTarget().getSkew();
        }
        return 0.0;
    }
    

    // Returns pitch (up/down angle), or 0 if no target
    public double getPitch() {
        if (hasTarget()) {
            return result.getBestTarget().getPitch();
        }
        return 0.0;
    }
    

    // Returns AprilTag ID, or -1 if no target
    public int getTagID() {
        if (!hasTarget()) return -1;
        return target.getFiducialId();
    }

}
