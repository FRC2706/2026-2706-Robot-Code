// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StartShooter extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem m_ShooterSubystsem;
  private int distance;

  /**
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartShooter(ShooterSubsystem subsystem, int position) {
    m_ShooterSubystsem = subsystem;
    distance = position;
    addRequirements(m_ShooterSubystsem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubystsem.isRPMinRange(distance)) {
      m_ShooterSubystsem.ready(distance);
    } else {
      m_ShooterSubystsem.spinningUp(distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_ShooterSubystsem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}