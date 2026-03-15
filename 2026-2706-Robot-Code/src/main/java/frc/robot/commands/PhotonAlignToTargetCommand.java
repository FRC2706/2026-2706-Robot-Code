// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.UtilityConstants.VisionConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that rotates the robot to face an AprilTag while allowing the driver
 * to control translation. Rotation is handled automatically when a tag is
 * visible, using a simple proportional controller on the tag yaw.
 */
public class PhotonAlignToTargetCommand extends Command {
  private final PhotonSubsystem m_photonSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;

  // Goal distance from the AprilTag (in meters) (currently unused)
  private final double goalDistance;

  // Gains
  private static final double kYawGain = 0.05; // units: (rad/s) per radian of yaw error
  private static final double kDeadbandDeg = 1.0; // degrees

  // Max automatic angular velocity (rad/s)
  private static final double kMaxAutoOmegaRadPerSec = 0.3;

  // Swerve drive / joystick inputs
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
    m_Vx = Vx;
    m_Vy = Vy;
    m_Omega = omega;
    m_DriveDeadband = driveDeadband;
    m_AngleDeadband = angleDeadband;
    this.goalDistance = VisionConstants.kGoalDistance;

    addRequirements(photonSubsystem, swerveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("PhotonAlignToTargetCommand started (goalDistance = " + goalDistance + " m)");
  }

  @Override
  public void execute() {
    // Read joystick inputs
    double adjustedVx = m_Vx.getAsDouble();
    double adjustedVy = m_Vy.getAsDouble();
    double adjustedOmegaJoystick = m_Omega.getAsDouble();

    // Apply deadbands
    if (Math.abs(adjustedVx) < m_DriveDeadband) {
      adjustedVx = 0.0;
    }
    if (Math.abs(adjustedVy) < m_DriveDeadband) {
      adjustedVy = 0.0;
    }
    if (Math.abs(adjustedOmegaJoystick) < m_AngleDeadband) {
      adjustedOmegaJoystick = 0.0;
    }

    // Scale translation by max linear velocity
    Translation2d translation =
        new Translation2d(
            adjustedVx * m_swerveSubsystem.getMaximumChassisVelocity(),
            adjustedVy * m_swerveSubsystem.getMaximumChassisVelocity());

    double commandedOmegaRadPerSec;

    // If we don't see a target, use driver's rotation input (scaled to rad/s)
    if (!m_photonSubsystem.hasTarget()) {
      // joystick is in [-1..1] (assumption); multiply by maximum chassis angular velocity to produce rad/s
      commandedOmegaRadPerSec = adjustedOmegaJoystick * m_swerveSubsystem.getMaximumChassisAngularVelocity();
      m_swerveSubsystem.drive(translation, commandedOmegaRadPerSec, true);
      return;
    }

    // We have a target: compute yaw error from vision (degrees)
    double yawErrorDeg = m_photonSubsystem.getYaw();

    // If inside the deadband, don't auto-rotate
    if (Math.abs(yawErrorDeg) <= kDeadbandDeg) {
      // Hold rotation (or allow small joystick override - here we hold)
      commandedOmegaRadPerSec = 0.0;
    } else {
      // Convert yaw error (deg -> rad), apply proportional gain (gain units: rad/s per rad)
      double yawErrorRad = Math.toRadians(yawErrorDeg);
      double omegaFromVision = -yawErrorRad * kYawGain; // negative sign to reduce yaw error direction

      // Clamp to maximum automatic angular velocity
      commandedOmegaRadPerSec = Math.max(-kMaxAutoOmegaRadPerSec, Math.min(kMaxAutoOmegaRadPerSec, omegaFromVision));
    }

    // Drive with driver translations and auto-rotation (rad/s)
    m_swerveSubsystem.drive(translation, commandedOmegaRadPerSec, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, true);
    System.out.println("PhotonAlignToTargetCommand ended (interrupted=" + interrupted + ")");
  }

  @Override
  public boolean isFinished() {
    return false; // run until canceled
  }
}