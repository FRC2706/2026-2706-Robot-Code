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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

// Class
public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera"); //make sure this name matches the camera name in photonvision interface
     //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final double kCameraHeight = 0.44; // assigns camera height in meters
    private static final double kTargetHeight = 1.22; // assigns target height in meters
    private static double kCameraPitch = 30; // assigns camera angle in degrees
    public double m_planarDistance = 0;     
    public int m_roundedPlanarDistance = 0; // Rounded planar distance stored as int
    public Alliance currentAlliance = Alliance.Red;
    public PhotonSubsystem() {
    }

    @Override
    public void periodic() {
        if (camera1 != null) {
            result = camera1.getLatestResult();

            if (result.hasTargets()) {
                target = result.getBestTarget();
            // Get ID, Yaw, Pitch, Area, Pose Ambiguity (this is a backup/alternative method to the one we did below)
            int targetId = target.getFiducialId();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();


            }

            else {
                target = null;
            }

            // If no targets, skip calculations
            if (!result.hasTargets()) {
                target = null;
                return;
            }

            // Get the AprilTag's known field pose
            Optional<Pose3d> tagPoseOpt = kTagLayout.getTagPose(getTagID());
            if (tagPoseOpt.isEmpty()) {
                return;
            }
            Pose3d tagPose3d = tagPoseOpt.get();

            // Find the distance between the camera and the target in meters. Convert degrees to radians because that's what Math.tan expects.
            double denominator = Math.tan(Math.toRadians(target.getPitch()) + Math.toRadians(kCameraPitch));
            if (Math.abs(denominator) <= 0) {
                // sentinel for invalid / infinite distance
                m_planarDistance = 0;
            } 
            if (denominator == 0){
                m_planarDistance = 0;
            }
            else {
                double planar = (kTargetHeight - kCameraHeight) / denominator;
                // store as int (rounded) and as double (non-rounded)
                m_roundedPlanarDistance = (int) Math.round(planar);
                m_planarDistance = planar;
            }
        }
        else {
            System.out.println("camera1 is null");
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


// Returns AprilTag ID of the detected target, or -1 if no target (or if targets are not one of the 3 listed for each alliance)
public int getTagID() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    allianceOpt.ifPresent(a -> currentAlliance = a);
    currentAlliance = allianceOpt.get(); // Update current alliance before checking targets
    if (!hasTarget()) return -1;
    
    int targetId = target.getFiducialId();
    if (targetId < 0) return -1;
 
    if (currentAlliance == Alliance.Red) {
        // RED ALLIANCE CODE: only accept 8, 2, 10
        if (targetId == 8 || targetId == 2 || targetId == 10) {
            return targetId;
        } else {
            return -1;
        }
    } else if (currentAlliance == Alliance.Blue) {
        // BLUE ALLIANCE CODE: only accept 24, 25, 27
        if (targetId == 24 || targetId == 25 || targetId == 27) {
            return targetId;
        } else {
            return -1;
        }
    }

    return -1;
}

    // Returns PlanarDistance calculated in periodic
  public double getDistance() {
    if (m_planarDistance <= 0){
        return 0;
    }
    else {
        return m_planarDistance;
    }

  }

  // Returns the 3D slant distance (direct distance to AprilTag)
  public double getSlantDistance() {
    if (!hasTarget()) return 0.0;
    double heightDiff = kTargetHeight - kCameraHeight;
    return Math.hypot(m_planarDistance, heightDiff);
  }
}