package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopIndexerCommand extends Command{
    private final ShooterSubsystem m_ShooterSubsystem;

    public StopIndexerCommand(ShooterSubsystem shooterSubsystem){
        m_ShooterSubsystem = shooterSubsystem;

        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize(){
        m_ShooterSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
