// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class startShooter extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem m_ShooterSubystsem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public startShooter(ShooterSubsystem subsystem) {
    m_ShooterSubystsem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubystsem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubystsem.spinningUp();
    
    m_ShooterSubystsem.ready();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

