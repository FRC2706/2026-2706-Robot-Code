// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonSubsystem;

public class PhotonDistanceCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  public static PhotonSubsystem m_subsystem;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public PhotonDistanceCommand(PhotonSubsystem subsystem) {
    m_subsystem = subsystem;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_subsystem.getDistance();
    System.out.println(m_subsystem.getDistance());

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}