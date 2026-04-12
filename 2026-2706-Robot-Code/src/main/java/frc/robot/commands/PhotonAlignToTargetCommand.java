package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class PhotonAlignToTargetCommand extends Command {

    // Known hub position on the field (meters)
    private static final double kHubX = 4.619;
    private static final double kHubY = 4.027;

    
    private static final double kStandoffDistance = 1.5;


    private static final double kPosTolerance  = 0.10;
    private static final double kRotTolerance  = Math.toRadians(3.0);

    private final PhotonSubsystem  m_photon;
    private final SwerveSubsystem  m_swerve;

    private Command m_driveCommand = null;
    private boolean m_commandScheduled = false;

    public PhotonAlignToTargetCommand(PhotonSubsystem photon, SwerveSubsystem swerve) {
        m_photon = photon;
        m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_driveCommand     = null;
        m_commandScheduled = false;
        System.out.println("PhotonAlignToTargetCommand: starting");
    }

    @Override
    public void execute() {

     
        if (m_commandScheduled) return;

       
        if (!m_photon.hasTarget()) {
            m_swerve.drive(new Translation2d(0, 0), 0, false);
            return;
        }

        Optional<Pose3d> robotPoseOpt = m_photon.getEstimatedRobotPose();
        if (robotPoseOpt.isEmpty()) {
            m_swerve.drive(new Translation2d(0, 0), 0, false);
            return;
        }

        
        Pose3d robotPose3d = robotPoseOpt.get();
        double robotX = robotPose3d.getX();
        double robotY = robotPose3d.getY();

        
        double dx = kHubX - robotX;
        double dy = kHubY - robotY;
        double dist = Math.hypot(dx, dy);

        if (dist < 0.01) {
            
            return;
        }

        
        double ux = -dx / dist;
        double uy = -dy / dist;

        
        double targetX = kHubX + ux * kStandoffDistance;
        double targetY = kHubY + uy * kStandoffDistance;

       
        double facingAngle = Math.atan2(dy, dx); // points toward hub

        Pose2d approachPose = new Pose2d(
            new Translation2d(targetX, targetY),
            new Rotation2d(facingAngle)
        );

        System.out.printf("PhotonAlignToHubCommand: navigating to (%.2f, %.2f) facing %.1f deg%n",
                targetX, targetY, Math.toDegrees(facingAngle));

        
        m_driveCommand = m_swerve.driveToPose(approachPose);
        m_driveCommand.schedule();
        m_commandScheduled = true;
    }

    @Override
    public void end(boolean interrupted) {
        if (m_driveCommand != null) {
            m_driveCommand.cancel();
        }
        m_swerve.drive(new Translation2d(0, 0), 0, false);
        System.out.println("PhotonAlignToHubCommand: ended, interrupted=" + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (!m_commandScheduled || m_driveCommand == null) return false;

        
        if (m_driveCommand.isFinished()) return true;

        
        Pose2d current = m_swerve.getPose();
        double dx = current.getX() - kHubX;
        double dy = current.getY() - kHubY;
        double distToHub = Math.hypot(dx, dy);
        double distError = Math.abs(distToHub - kStandoffDistance);
        double headingToHub = Math.atan2(kHubY - current.getY(), kHubX - current.getX());
        double rotError = Math.abs(current.getRotation().getRadians() - headingToHub);

        return distError < kPosTolerance && rotError < kRotTolerance;
    }
}