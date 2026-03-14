// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Imports
import edu.wpi.first.wpilibj2.command.Command;
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
    
    // Calculate rotation velocity: proportional to yaw error
    // Negative yaw = target to the left, positive yaw = target to the right
    // Convert yaw from degrees to radians for rotation velocity (rad/s)
    double rotationVelocity = -Math.toRadians(yawError) * kYawGain;
    
    // Clamp rotation velocity (in rad/s)
    rotationVelocity = Math.max(-0.3, Math.min(0.3, rotationVelocity));
    
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