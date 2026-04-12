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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.math.geometry.Rotation3d;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Known hub position on the field (meters). Z is stored but only X/Y used for planar distance.
    public static final double kHubX = 4.619;
    public static final double kHubY = 4.027;
    public static final double kHubZ = 1.13;

    public double m_planarDistance = 0;
    public double m_lastKnownDistance = 0;
    public Alliance currentAlliance = Alliance.Blue;

    // NetworkTables entries for sharing vision data
    private final DoublePublisher m_planarDistanceEntry;
    private final BooleanPublisher m_hasTargetEntry;
    private final DoublePublisher m_lastKnownDistanceEntry;

    private static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(0.001, 0.232, 0.45),
        new Rotation3d(0.0, -Math.toRadians(30), 0.0)
    );

    public PhotonSubsystem() {
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("datatable");
        m_planarDistanceEntry = networkTable.getDoubleTopic("planarDistanceMeters").publish();
        m_hasTargetEntry = networkTable.getBooleanTopic("hasTarget").publish();
        m_lastKnownDistanceEntry = networkTable.getDoubleTopic("lastKnownDistanceMeters").publish();
    }

    @Override
    public void periodic() {
        target = null;

        try {
            result = camera1.getLatestResult();

            if (result.hasTargets()) {
                target = result.getBestTarget();

                // Compute distance using the robot's estimated field pose vs. the
                // known hub position. This removes the AprilTag offset problem
                // because we're measuring robot→hub in field coordinates, not
                // camera→tag in camera coordinates.
                Optional<Pose3d> robotPoseOpt = getEstimatedRobotPose();
                if (robotPoseOpt.isPresent()) {
                    Pose3d robotPose = robotPoseOpt.get();
                    double dx = kHubX - robotPose.getX();
                    double dy = kHubY - robotPose.getY();
                    m_planarDistance = Math.hypot(dx, dy);
                } else {
                    // Fallback: raw camera-to-tag planar distance if pose estimation fails
                    Translation3d camTranslation = target.getBestCameraToTarget().getTranslation();
                    m_planarDistance = Math.hypot(camTranslation.getX(), camTranslation.getY());
                }

                m_lastKnownDistance = m_planarDistance;
            }
        } catch (Exception e) {
            System.out.println(e);
        }

        // Publish to NetworkTables after state is updated
        m_planarDistanceEntry.set(m_planarDistance);
        m_hasTargetEntry.set(hasTarget());
        m_lastKnownDistanceEntry.set(m_lastKnownDistance);
    }

    public boolean hasTarget() {
        return result != null && result.hasTargets();
    }

    public double getYaw() {
        if (hasTarget()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public double getSkew() {
        if (hasTarget()) {
            return result.getBestTarget().getSkew();
        }
        return 0.0;
    }

    public double getPitch() {
        if (hasTarget()) {
            return result.getBestTarget().getPitch();
        }
        return 0.0;
    }

    public Transform3d cameraToTarget() {
        if (!hasTarget()) return new Transform3d();
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

    public int getTagID() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        allianceOpt.ifPresent(a -> currentAlliance = a);

        if (!hasTarget()) return -1;

        int targetId = target.getFiducialId();
        if (targetId < 0) return -1;

        if (currentAlliance == Alliance.Red) {
            if (targetId == 10) {
                return targetId;
            } else {
                return -1;
            }
        } else if (currentAlliance == Alliance.Blue) {
            if (targetId == 9 || targetId == 10 || targetId == 11) {
                return targetId;
            } else {
                return -1;
            }
        }

        return -1;
    }

    public void setLastKnownDistance(double fallbackDistance) {
        if (hasTarget() && m_planarDistance >= 0) {
            m_lastKnownDistance = m_planarDistance;
        } else {
            m_lastKnownDistance = fallbackDistance;
        }
    }

    /**
     * Returns the planar (X/Y) distance from the robot to the hub center.
     * When a tag is visible, this is computed from the robot's estimated field pose
     * vs. the known hub coordinates — not from the raw camera-to-tag vector.
     * Falls back to the last known distance when no tag is visible.
     */
    public double getDistance() {
        if (hasTarget()) {
            return m_planarDistance;
        }
        return m_lastKnownDistance;
    }

    /**
     * Returns the 3D slant distance from the robot to the hub center,
     * accounting for the height difference between the robot (ground level, Z=0)
     * and the hub (Z = kHubZ).
     */
    public double getSlantDistance() {
        double planar = getDistance();
        if (planar <= 0.0) return 0.0;
        return Math.hypot(planar, kHubZ);
    }
}