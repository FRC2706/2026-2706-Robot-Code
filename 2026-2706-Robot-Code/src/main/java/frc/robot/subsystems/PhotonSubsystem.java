// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
// Class
public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1; //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final double kCameraHeight = 0.44; // assigns camera height in meters
    private static final double kTargetHeight = 1.22; // assigns target height in meters // TODO: change this so it changes based on APrilTag ID
    private static double kCameraPitch = 30; // assigns camera angle in radians
    public double m_planarDistance = 0;       

    public PhotonSubsystem() { //private? or public?
        camera1 = new PhotonCamera("Arducam"); //make sure this name matches the camera name in photonvision interface
    }

    @Override
    public void periodic() {
        if (camera1 != null) {
            result = camera1.getLatestResult();

            if (result.hasTargets()) {
                target = result.getBestTarget();
            }

            else {
                target = null;
            }

            // If no targets, skip calculations
            if (!result.hasTargets()) {
                target = null;
                return;
            }

            // We have a target
            target = result.getBestTarget();

            // Get the AprilTag's known field pose
            Optional<Pose3d> tagPoseOpt = kTagLayout.getTagPose(getTagID());
            if (tagPoseOpt.isEmpty()) {
                System.out.println("Tag pose not found in layout for ID " + getTagID());
                return;
            }
            Pose3d tagPose3d = tagPoseOpt.get();
            
            m_planarDistance = (kTargetHeight-kCameraHeight)/(Math.tan((Math.PI/180*getPitch())+(Math.PI/180*kCameraPitch)));
        }
        else {
            target = null;
            result = null;
        }
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

    // Returns cameraToTarget transform3d things??
    public Transform3d cameraToTarget() {
        if (!hasTarget()) return new Transform3d();
        return new Transform3d(target.getBestCameraToTarget().getTranslation(), target.getBestCameraToTarget().getRotation());
    }

    // Returns AprilTag ID, or -1 if no target
    public int getTagID() {
        if (!hasTarget()) return -1;
        return target.getFiducialId();
    }
    // Returns planarDistance calculated in periodic
  public double getDistance() {
        return m_planarDistance;
  }

  // Returns the 3D slant distance (direct distance to AprilTag)
  public double getSlantDistance() {
    if (!hasTarget()) return 0.0;
    double heightDiff = kTargetHeight - kCameraHeight;
    return Math.hypot(m_planarDistance, heightDiff);
  }
}