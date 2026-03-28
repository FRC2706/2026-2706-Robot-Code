package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.UtilityConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAgitateCommand extends Command{
    private final IntakeSubsystem m_IntakeSubsystem;
    private double timeSinceArmDown = Timer.getFPGATimestamp();
    private double timeSinceArmUp = Timer.getFPGATimestamp();
    private double timeSinceRollerChange = Timer.getFPGATimestamp();
    private boolean isAtMid = false;
    private boolean isRollerOn = false;

    public IntakeAgitateCommand(IntakeSubsystem intakeSubsystem){
        m_IntakeSubsystem = intakeSubsystem;

        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize(){
        m_IntakeSubsystem.startIntake();
        m_IntakeSubsystem.moveIntakeMid();
        isAtMid = true;
        isRollerOn = true;
    }

    @Override
    public void execute(){
        double time = Timer.getFPGATimestamp();

        //Every few seconds, move the intake up and down
        if (time - timeSinceArmDown >= UtilityConstants.RobotConstants.kAgitateArmTimeMid && !isAtMid){
            timeSinceArmUp = time;
            isAtMid = true;            
            m_IntakeSubsystem.moveIntakeMid();  
        }
        else if (time - timeSinceArmUp >= UtilityConstants.RobotConstants.kAgitateArmTimeDown && isAtMid){
            timeSinceArmDown = time;
            m_IntakeSubsystem.moveIntakeUp();
            isAtMid = false;
        }


        //Every few seconds, turn the intake roller on and off
        if (time - timeSinceRollerChange >= UtilityConstants.RobotConstants.kAgitateRollerTime){
            timeSinceRollerChange = time;

            if (isRollerOn){
                m_IntakeSubsystem.stopIntake();
                isRollerOn = false;
            }
            else{
                m_IntakeSubsystem.startIntake();
                isRollerOn = true;
            }

        }
        
    }

    @Override
    public void end(boolean interrupted){
        m_IntakeSubsystem.stopIntake();
        m_IntakeSubsystem.moveIntakeUp();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
