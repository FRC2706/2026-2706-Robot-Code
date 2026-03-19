package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ClearIndexerCommand extends Command{
    private final ShooterSubsystem m_ShooterSubsystem;

    public ClearIndexerCommand(ShooterSubsystem shooterSubsystem){
        m_ShooterSubsystem = shooterSubsystem;

        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.clearIndexer();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
