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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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

    public static final double shooterToRobotDist = 0;
    public static double shooterToHubDist = 0;
    public static double robotAlignmentYaw = 0;
    public final SwerveSubsystem m_SwerveSubsystem;


    //4.6 x hub
    //4.027 y hub

    public PhotonSubsystem(SwerveSubsystem swerveSubsystem) {
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("datatable");
        m_planarDistanceEntry = networkTable.getDoubleTopic("planarDistanceMeters").publish();
        //m_roundedPlanarDistanceEntry = networkTable.getDoubleTopic("planarDistanceRounded").publish();
        m_hasTargetEntry = networkTable.getBooleanTopic("hasTarget").publish();
        m_lastKnownDistanceEntry = networkTable.getDoubleTopic("lastKnownDistanceMeters").publish();
        m_SwerveSubsystem = swerveSubsystem;
    }

    @Override
     public void periodic() {

        try{
            // Publish values to NetworkTables for external consumers (defensive, don't throw)
            m_planarDistanceEntry.set(shooterToHubDist);
            
            calculateYawToHub(m_SwerveSubsystem.getPose(), new Pose2d(4.6,4.027,new Rotation2d(0)));
            
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
         return new Transform3d(target.getBestCameraToTarget().getTranslation(), target.getBestCameraToTarget().getRotation());
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

    //Calculates the optimal robot yaw and distance from the camera to the center of the hubgiven the robot position, hub position. Must be field relative
    public void calculateYawToHub(Pose2d robotPose, Pose2d hubPose){
        /*----------------- Finding distances -----------------*/
        //Distance from center of robot to center of hub
        double robotToHubDist;

        //Calculate straightline distance using pythagorean theorem
        robotToHubDist = Math.sqrt(Math.pow(robotPose.getX() - hubPose.getX(),2) + Math.pow(robotPose.getY() - hubPose.getY(),2));

        //Calculate the distance from the shooter (front edge) to the hub using pythagorean theorem. Constant subtraction is the distance from 
        //the center of the shooter to the camera
        shooterToHubDist = Math.sqrt(Math.pow(robotToHubDist,2) - Math.pow(shooterToRobotDist,2)) - 0;

        /*------------------ Finding yaw ---------------------- */
        //Finding the angle between straightline distance of robot to hub and the projected position of the shooter
        double alpha = Math.acos(shooterToRobotDist/robotToHubDist);
        double omega = Math.toRadians(90) - alpha;

        //Finding which quadrant the center of the hub is relative to the shooter
        byte quadrant = 0;
        if (robotPose.getX() > hubPose.getX()){
            if (robotPose.getY() > hubPose.getY()){
                quadrant = 4;
            }
            else{
                quadrant = 3;
            }
        }
        else{
            if (robotPose.getY() > hubPose.getY()){
                quadrant = 1;
            }else{
                quadrant = 2;
            }
        }

        //Finding angle from zero on the robot the the straightline distance to the hub
        double theta = Math.acos(Math.abs(robotPose.getX() - hubPose.getX())/robotToHubDist);
        
        theta = Math.toDegrees(theta);
        omega = Math.toDegrees(omega);

        //Calculate target yaw
        switch (quadrant){
            case 1:
                {
                    robotAlignmentYaw = -(theta - omega);
                }
                break;
            case 2:
                {
                    robotAlignmentYaw = theta - omega;
                }
                break;
            case 3:
                {
                    robotAlignmentYaw = -180 + (theta - omega);
                }
                break;
            case 4:
                {
                    robotAlignmentYaw = 90 + (theta - omega);   
                }
                break;
            default:
                robotAlignmentYaw = 0;
        }

        System.out.println(robotAlignmentYaw);
    }
}