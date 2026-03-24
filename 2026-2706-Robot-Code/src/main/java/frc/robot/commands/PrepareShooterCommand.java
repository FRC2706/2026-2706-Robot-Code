package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareShooterCommand extends Command{
    
    private final ShooterSubsystem m_ShooterSubsystem;
    private final int position;

    public PrepareShooterCommand(ShooterSubsystem shooterSubsystem, int position){
        
        m_ShooterSubsystem = shooterSubsystem;

        this.position = position;

        addRequirements(m_ShooterSubsystem);

    }

    @Override
    public void execute(){
        m_ShooterSubsystem.spinningUp(position);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
