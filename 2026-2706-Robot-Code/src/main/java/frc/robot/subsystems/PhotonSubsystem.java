// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import static edu.wpi.first.units.Units.Value;

import java.util.Optional;

// Imports
import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
//import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
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
    //private static double kTargetPitch = Math.PI/2; // assigns target angle in radians
    //private final SwerveSubsystem m_SwerveSubsystem;
    private double m_planarDistance;            
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
        //System.out.println("Pitch:" + getPitch());
        //System.out.println("Yaw:" + getYaw());
        //System.out.println("Skew" + getSkew());
        //System.out.println("Apriltag: " + getTagID());

    // (removed unused camera-relative targetPose) — using known field tag pose for estimation instead
        // If no targets, skip calculations
        if (!result.hasTargets()) {
            System.out.println("No target found");
            target = null;
            return;
        }

        // We have a target
        target = result.getBestTarget();
        System.out.println(target);

        // Get the AprilTag's known field pose
        Optional<Pose3d> tagPoseOpt = kTagLayout.getTagPose(getTagID());
        if (tagPoseOpt.isEmpty()) {
            System.out.println("Tag pose not found in layout for ID " + getTagID());
            return;
        }
        Pose3d tagPose3d = tagPoseOpt.get();
        /*Pose2d fieldTagPose2d = tagPose3d.toPose2d();

        // Camera->target transform observed by PhotonVision
        Transform3d camToTarget3d = cameraToTarget();
        Transform2d camToTarget2d = new Transform2d(camToTarget3d.getTranslation().toTranslation2d(), camToTarget3d.getRotation().toRotation2d());

        // If PhotonVision didn't provide a valid 3D translation, warn (Z=0 is a common sign)
        if (Math.abs(camToTarget3d.getTranslation().getZ()) < 1e-6
            && Math.abs(camToTarget3d.getTranslation().getX()) < 1e-6
            && Math.abs(camToTarget3d.getTranslation().getY()) < 1e-6) {
            System.out.println("Warning: PhotonVision provided zero camera->target translation; pose estimation may be unavailable.\nCheck pipeline tag size and camera calibration.");
        }

        // Estimate robot pose on the field using known tag pose and observed camera->target
        // Use the tag pose's actual Z value for the target height and a zero target pitch
        // (tags are vertical; use 0 radians for pitch unless you have a different tag orientation).
        
        /*Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            kCameraHeight,
            tagPose3d.getZ(), // use actual tag height from the field layout
            kCameraPitch,
            0.0, // tag pitch in radians (use 0 for standard vertical AprilTags)
            Rotation2d.fromDegrees(-target.getYaw()),
            m_SwerveSubsystem.getOdometryHeading(),
            fieldTagPose2d,
            camToTarget2d);

        // Explicit planar distance (robot XY to tag XY)
       double dx = robotPose.getX() - fieldTagPose2d.getX();
       double dy = robotPose.getY() - fieldTagPose2d.getY();*/
       
        m_planarDistance = (kTargetHeight-kCameraHeight)/(Math.tan((Math.PI/180*getPitch())+(Math.PI/180*kCameraPitch)));
        // Slant distance using camera and tag heights
        //double heightDiff = kCameraHeight - tagPose3d.getZ();
        //double slantDistance = Math.hypot(m_planarDistance, heightDiff);
        //System.out.println("Planar distance (robot to tag, field XY) [m]: " + m_planarDistance);
        //System.out.println("Slant distance (approx, includes height difference) [m]: " + slantDistance);
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