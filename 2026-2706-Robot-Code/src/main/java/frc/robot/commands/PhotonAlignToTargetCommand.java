package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UtilityConstants.VisionConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

/** Command that rotates the robot to face an AprilTag while allowing the driver
 * to control translation. Rotation is handled automatically when a tag is
 * visible, using a simple proportional controller on the tag yaw.
 */
public class PhotonAlignToTargetCommand extends Command {
  private final PhotonSubsystem m_photonSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  private final PhotonCamera m_camera;
  private final PIDController m_controller;

  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private final double goalDistance;

  private static final double kYawGain = 0.05;
  private static final double kDeadbandDeg = 1.0;
  private static final double kMaxAutoOmegaRadPerSec = 0.3;

  private final DoubleSupplier m_Vx;
  private final DoubleSupplier m_Vy;
  private final DoubleSupplier m_Omega;
  private final double m_DriveDeadband;
  private final double m_AngleDeadband;

  public PhotonAlignToTargetCommand(
      PhotonSubsystem photonSubsystem,
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier Vx,
      DoubleSupplier Vy,
      DoubleSupplier omega,
      double driveDeadband,
      double angleDeadband) {
    m_photonSubsystem = photonSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_camera = photonSubsystem.getCamera();
    m_controller = new PIDController(kP, kI, kD);

    m_Vx = Vx;
    m_Vy = Vy;
    m_Omega = omega;
    m_DriveDeadband = driveDeadband;
    m_AngleDeadband = angleDeadband;
    this.goalDistance = VisionConstants.kGoalDistance;

    addRequirements(photonSubsystem);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // Initialization quieted to avoid console spam during operation.
  }

  @Override
  public void execute() {
    double adjustedVx = m_Vx.getAsDouble();
    double adjustedVy = m_Vy.getAsDouble();
    double adjustedOmegaJoystick = m_Omega.getAsDouble();

    if (Math.abs(adjustedVx) < m_DriveDeadband) {
      adjustedVx = 0.0;
    }
    if (Math.abs(adjustedVy) < m_DriveDeadband) {
      adjustedVy = 0.0;
    }
    if (Math.abs(adjustedOmegaJoystick) < m_AngleDeadband) {
      adjustedOmegaJoystick = 0.0;
    }

    Translation2d translation = new Translation2d(
        adjustedVx * m_swerveSubsystem.getMaximumChassisVelocity(),
        adjustedVy * m_swerveSubsystem.getMaximumChassisVelocity());

    double commandedOmegaRadPerSec;

    if (!m_photonSubsystem.hasTarget()) {
      commandedOmegaRadPerSec = adjustedOmegaJoystick * m_swerveSubsystem.getMaximumChassisAngularVelocity();
      m_swerveSubsystem.drive(translation, commandedOmegaRadPerSec, true);
      return;
    }

    double yawErrorDeg = m_photonSubsystem.getYaw();

    if (Math.abs(yawErrorDeg) <= kDeadbandDeg) {
      commandedOmegaRadPerSec = 0.0;
    } else {
      double yawErrorRad = Math.toRadians(yawErrorDeg);
      double omegaFromVision = -yawErrorRad * kYawGain;
      commandedOmegaRadPerSec = Math.max(-kMaxAutoOmegaRadPerSec, Math.min(kMaxAutoOmegaRadPerSec, omegaFromVision));
    }

    m_swerveSubsystem.drive(translation, commandedOmegaRadPerSec, true);

    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      double rotationSpeed = m_controller.calculate(result.getBestTarget().getYaw(), 0);
      m_swerveSubsystem.drive(translation, rotationSpeed, false);
    } else {
      m_swerveSubsystem.drive(translation, 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
