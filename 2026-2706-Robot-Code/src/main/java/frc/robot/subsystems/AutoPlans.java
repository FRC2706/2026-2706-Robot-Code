package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

// For testing
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoPlans extends SubsystemBase {

    PathPlannerAuto DriveForward;

    public AutoPlans(){
        registerCommands();
    }

    public void registerCommands(){
        NamedCommands.registerCommand("test",new PrintCommand("test"));
    }

    public Command getAutonomousCommand(int commandIndex){
        switch(commandIndex){
            default:
                return null;
            case 0:
    
                if (DriveForward == null) {
                    try {
                        DriveForward = new PathPlannerAuto("Drive_Forward");
                    } catch (Exception e) {
                        // If creation fails, return a harmless fallback command and print the error.
                        e.printStackTrace();
                        return new PrintCommand("Failed to load Drive_Forward auto");
                    }
                }
                return DriveForward;
        }
    }
    
}
