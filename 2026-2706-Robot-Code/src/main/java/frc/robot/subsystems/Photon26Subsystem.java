package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photon26Subsystem extends SubsystemBase {
    private PhotonTrackedTarget target;
    private int tagId = -1;

    // Replace these zeros with camera's position relative to the robot origin.
    // Translation: meters (forward, left, up).
    // Rotation: radians (roll, pitch, yaw). Use Math.toRadians() for degrees.
    private final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    private final Transform3d cameraToRobot = robotToCamera.inverse();

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // This must match the camera name in the PhotonVision interface.
    private final PhotonCamera camera =
        new PhotonCamera("Arducam_OV9281_USB_Camera");

    public Photon26Subsystem() {
        camera.setPipelineIndex(0);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            target = result.getBestTarget();
            tagId = target.getFiducialId();

            Transform3d cameraToTarget = target.getBestCameraToTarget();
            double xDistance = cameraToTarget.getX();
            double yaw = target.getYaw();

            System.out.println("TagId: " + tagId+ " | Range: " + xDistance+ "m | Yaw: " + yaw);
        } else {
            target = null;
            tagId = -1;
        }
    }

    // Returns an empty Optional if no target or known field tag is available.
    public Optional<Pose3d> getRobotPosition() {
        if (target == null) {
            return Optional.empty();
        }

        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);

        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
            PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                tagPose.get(),
                cameraToRobot
            )
        );
    }
}