package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Imports
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.UtilityConstants.VisionConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// Class
/** An example command that uses an example subsystem. */
public class PhotonAlignToTargetCommand extends Command {
  private final PhotonSubsystem m_photonSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  private static final double kDeadbandDeg = 1.0; // degrees within which we consider aligned

  // PID/Proportional gains for distance control
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
    addRequirements(photonSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we can see the target
    double yawErrorDeg = m_photonSubsystem.robotAlignmentYaw - m_swerveSubsystem.getYaw();

    double rotationVelocity = 0;

    if (Math.abs(yawErrorDeg) > kDeadbandDeg){
      rotationVelocity = yawErrorDeg * kYawGain;

      //Clamp rotation velocity
      rotationVelocity = Math.max(-30, Math.min(30,rotationVelocity));
    }
    else{
      rotationVelocity = 0;
    }
    
    // Drive the robot (rotate in place to face the tag)
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), rotationVelocity, false);
    
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}