package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.units.Units.Value;

import java.util.Optional;

// Imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PhotonSubsystem;

// Class
/** An example command that uses an example subsystem. */
public class PhotonDistanceCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final PhotonSubsystem m_subsystem;

  /**
   * Creates a new PhotonGetDistance.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PhotonDistanceCommand(PhotonSubsystem subsystem) {
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
  // Called once the command ends or is interrupted.
    m_subsystem.getDistance();

  }

  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return false;
  }
}
