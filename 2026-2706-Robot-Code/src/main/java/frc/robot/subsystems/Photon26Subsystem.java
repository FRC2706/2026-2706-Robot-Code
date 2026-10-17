package frc.robot.subsystems;

// imports
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Photon26Subsystem extends SubsystemBase{
    private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); //make sure this name matches the camera name in photonvision interface
    
    public Photon26Subsystem() {
        camera.setPipelineIndex(0); // sets camera to use first pipeline in the list
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget(); // get best target in view

            int tagId = target.getFiducialId();

            Transform3d cameraToTarget = target.getBestCameraToTarget(); //3d position relative to camera lens

            // info from april tag data
            double xDistance = cameraToTarget.getX();
            double yDistance = cameraToTarget.getY();
            double zDistance = cameraToTarget.getZ();

            double yaw = target.getYaw();

            System.out.println("Seeing TagId: " + tagId + " | Range: " + xDistance + "m | Yaw: " + yaw);
        }
    }
}
