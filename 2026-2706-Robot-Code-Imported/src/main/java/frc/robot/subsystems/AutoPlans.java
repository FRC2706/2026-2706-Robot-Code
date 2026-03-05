package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoPlans extends SubsystemBase {


    PathPlannerAuto DriveForward;

    public AutoPlans(){
        registerCommands();
        
        
    }

    public void registerCommands(){
    }

    public String getAutonomousCommand(int commandIndex){
        switch(commandIndex){
            default:
                return null;
            case 0:
    
                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Drive Forward";
        }
    }
    
}
