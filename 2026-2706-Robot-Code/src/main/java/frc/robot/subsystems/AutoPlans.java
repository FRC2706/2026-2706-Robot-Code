package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.PrintCommand;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;

public class AutoPlans extends SubsystemBase {

    PathPlannerAuto DriveForward;

    /**
     * Construct AutoPlans and register any named PathPlanner commands that autos may call.
     * Accepts the IntakeSubsystem so intake commands can be created with the proper subsystem
     * instance.
     */
    public AutoPlans(IntakeSubsystem intake){
        registerCommands(intake);
    }

    /** Register PathPlanner named commands for intake actions. */
    public void registerCommands(IntakeSubsystem intake){
        try {
            Map<String, Command> eventMap = new HashMap<>();
            eventMap.put("IntakeUp", new IntakeUpCommand(intake));
            eventMap.put("IntakeDown", new IntakeDownCommand(intake));
            eventMap.put("IntakeOn", new RunIntakeCommandForward(intake));
            eventMap.put("IntakeReverse", new RunIntakeCommandReversed(intake));
            eventMap.put("IntakeOff", new RunIntakeCommandForward(intake).withTimeout(0));

            NamedCommands.registerCommands(eventMap);
        } catch (Throwable t) {
            System.out.println("Failed to register PathPlanner named commands: " + t.toString());
        }
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
            case 1:

                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Do Nothing";
            case 2:

                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Hub Auto";
            case 3:

                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Position 1 auto";
            case 4:

                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Position 2 auto";
            case 5:

                if (DriveForward == null) {
                    System.out.println("Failed to load Drive_Forward auto");
                    return null;
                }
                return "Position 3 auto";
        }
    }
    
}
