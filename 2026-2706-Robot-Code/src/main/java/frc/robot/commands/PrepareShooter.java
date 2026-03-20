package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class PrepareShooter extends Command{
  private final ShooterSubsystem m_ShooterSubystsem;
  private final int m_position;

  /**
   * @param shooterSubsystem The ShooterSubsystem used by this command.
   * @param position The position of the robot
   */
  public PrepareShooter(ShooterSubsystem shooterSubsystem, int position) {
    m_ShooterSubystsem = shooterSubsystem;
    m_position = position;
    addRequirements(m_ShooterSubystsem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubystsem.spinningUp(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Do NOT stop the motor once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}