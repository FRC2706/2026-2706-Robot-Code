package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// Class
/** An example command that uses an example subsystem. */
public class PhotonAlignToTargetCommand extends Command {
  private final PhotonSubsystem m_photonSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  
  // Goal distance from the AprilTag (in meters)
  private final double goalDistance;
  
  // PID/Proportional gains for distance control
  private static final double kDistanceGain = 0.5; // Proportional gain for forward/backward movement
  private static final double kYawGain = 0.05; // Proportional gain for rotation

  /**
   * Creates a new PhotonAlignToTargetCommand.
   *
   * @param photonSubsystem The PhotonVision subsystem
   * @param swerveSubsystem The swerve drive subsystem
   */
  public PhotonAlignToTargetCommand(PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem) {
    m_photonSubsystem = photonSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    this.goalDistance = VisionConstants.kGoalDistance;
    
    addRequirements(photonSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonAlignToTargetCommand started with goal distance: " + goalDistance + " m");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we can see the target
    if (!m_photonSubsystem.hasTarget()) {
      System.out.println("No target detected");
      // Stop the robot when no target is visible
      m_swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
      return;
    }
    
    // Get yaw error (left/right)
    double yawError = m_photonSubsystem.getYaw();
    
    // Get current distance to the AprilTag (3D slant distance)
    double currentDistance = m_photonSubsystem.getSlantDistance();
    
    // Calculate distance error (positive means we're too far, negative means we're too close)
    double distanceError = currentDistance - goalDistance;
    
    // Calculate forward velocity: positive = forward (away from target), negative = backward (toward target)
    // We want to reduce the distance error, so if error is positive (too far), we move backward (negative)
    double forwardVelocity = -distanceError * kDistanceGain;
    
    // Clamp the forward velocity to prevent excessive speed
    forwardVelocity = Math.max(-0.5, Math.min(0.5, forwardVelocity));
    
    // Calculate rotation velocity: proportional to yaw error
    // Negative yaw = target to the left, positive yaw = target to the right
    // Convert yaw from degrees to radians for rotation velocity (rad/s)
    double rotationVelocity = -Math.toRadians(yawError) * kYawGain;
    
    // Clamp rotation velocity (in rad/s)
    rotationVelocity = Math.max(-0.3, Math.min(0.3, rotationVelocity));
    
    // Drive the robot
    // Using robot-relative movement: forward/backward along robot's current heading
    // This allows the robot to move toward/away from the tag while rotating to face it
    m_swerveSubsystem.drive(
      new Translation2d(forwardVelocity, 0.0), // Forward/backward movement (robot-relative)
      rotationVelocity, // Rotation to keep the robot pointed at the target
      false // Robot-relative (not field-relative)
    );
    
    // Debug output
    System.out.println("Yaw: " + yawError + " deg | Current Distance: " + currentDistance + " m | Goal: " + goalDistance + " m | Error: " + distanceError + " m");
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    System.out.println("PhotonAlignToTargetCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
