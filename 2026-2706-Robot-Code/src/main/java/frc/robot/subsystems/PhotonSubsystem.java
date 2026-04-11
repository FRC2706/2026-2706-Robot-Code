// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Imports
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

// Class
public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera"); //make sure this name matches the camera name in photonvision interface
     //declares new camera object, not sure if it should be private or private final
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final double kCameraHeight = 0.45; // assigns camera height in meters
    private static final double kTargetHeight = 1.13; // assigns target height in meters
    private static double kCameraPitch = Math.toRadians(30); // assigns camera angle in degrees
    public double m_planarDistance = 0;     
    //public int m_roundedPlanarDistance = 0; // Rounded planar distance stored as int
    public double m_lastKnownDistance = 0; // Last known distance stored as double
    public Alliance currentAlliance = Alliance.Blue;
    private boolean printed_state = false; // Flag to track if "camera1 is null" has been printed
    // NetworkTables entries for sharing vision data
    private final DoublePublisher m_planarDistanceEntry;
    //private final DoublePublisher m_roundedPlanarDistanceEntry;
    private final BooleanPublisher m_hasTargetEntry;
    private final DoublePublisher m_lastKnownDistanceEntry;

    /** Transform describing the robot->camera mounting pose. Update values to match your robot's camera mount.
     * By default this is the identity transform (camera coincident with robot origin). */
    private static final Transform3d kRobotToCamera = new Transform3d();

    public PhotonSubsystem() {
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("datatable");
        m_planarDistanceEntry = networkTable.getDoubleTopic("planarDistanceMeters").publish();
        //m_roundedPlanarDistanceEntry = networkTable.getDoubleTopic("planarDistanceRounded").publish();
        m_hasTargetEntry = networkTable.getBooleanTopic("hasTarget").publish();
        m_lastKnownDistanceEntry = networkTable.getDoubleTopic("lastKnownDistanceMeters").publish();
    }

    @Override
     public void periodic() {

        try{
        // Publish values to NetworkTables for external consumers (defensive, don't throw)
        m_planarDistanceEntry.set(m_planarDistance);
        //m_roundedPlanarDistanceEntry.set(m_roundedPlanarDistance);
        m_hasTargetEntry.set(hasTarget());
        m_lastKnownDistanceEntry.set(m_lastKnownDistance);

      
            printed_state = false; // reset printed state when camera is available
             result = camera1.getLatestResult();

            if (result.hasTargets()) {
                 target = result.getBestTarget();

             }
    
                // Use the measured camera->target translation from PhotonVision for accurate distances.
                Translation3d camTranslation = target.getBestCameraToTarget().getTranslation();
                // Planar distance (ground-plane) — ignore vertical component
                m_planarDistance = Math.hypot(camTranslation.getX(), camTranslation.getY());
                // Last known planar distance
                m_lastKnownDistance = m_planarDistance;
                 
             }
             catch(Exception e){
                System.out.println(e);
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
         // PhotonTrackedTarget provides the measured camera->target transform directly
         return target.getBestCameraToTarget();
     }

    /**
     * Estimate the camera pose on the field using the detected AprilTag and the tag's known field pose.
     * Returns an empty Optional if there is no target or the tag id is not in the field layout.
     */
    public Optional<Pose3d> getEstimatedCameraPose() {
        if (!hasTarget()) return Optional.empty();
        int fiducialId = target.getFiducialId();
        if (fiducialId < 0) return Optional.empty();

        Optional<Pose3d> tagPoseOpt = kTagLayout.getTagPose(fiducialId);
        if (tagPoseOpt.isEmpty()) return Optional.empty();

        Pose3d tagPose = tagPoseOpt.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Transform3d targetToCamera = camToTarget.inverse();
        Pose3d cameraPose = tagPose.transformBy(targetToCamera);
        return Optional.of(cameraPose);
    }

    /**
     * Estimate the robot pose on the field by applying the inverse of the robot->camera mounting
     * transform to the estimated camera pose. Returns an empty Optional if the camera pose cannot
     * be estimated (no target, unknown tag id, etc.).
     */
    public Optional<Pose3d> getEstimatedRobotPose() {
        Optional<Pose3d> camPoseOpt = getEstimatedCameraPose();
        if (camPoseOpt.isEmpty()) return Optional.empty();
        Pose3d cameraPose = camPoseOpt.get();
        Pose3d robotPose = cameraPose.transformBy(kRobotToCamera.inverse());
        return Optional.of(robotPose);
    }


// // Returns AprilTag ID of the detected target, or -1 if no target (or if targets are not one of the 3 listed for each alliance)
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

 public void setLastKnownDistance(double fallbackDistance) {
     if (hasTarget() && m_planarDistance >= 0) { // only use measurement if there is a valid target and distance is non-negative
         m_lastKnownDistance = (double) m_planarDistance; // store the last known distance as a double for more precise fallback calculations
     } 
     else {
         m_lastKnownDistance = fallbackDistance;
     }
 }

   public double getDistance() {
     // If there's no detected target, distance is set to the last known value, but if there has never been a target, return 0
     if (hasTarget() == true) {
         return m_planarDistance;
     }
         return m_lastKnownDistance;
   }

   // Returns the 3D slant distance (direct distance to AprilTag)
   public double getSlantDistance() {
     if (!hasTarget()) return 0.0;
     double heightDiff = kTargetHeight - kCameraHeight;
     return Math.hypot(m_planarDistance, heightDiff);
}
}