package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.HashMap;
import java.util.Map;

import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.PrepareShooterCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShooter;
import frc.robot.UtilityConstants.shooterConstants.shooterPositions;

public class AutoPlans extends SubsystemBase {
    // Position 1 Start State: Vel = 0, Rot = -90, x = 3.505, y = 6.344, heading = -90.148, NCL = 0.825
    // Scoring Position 1 Start State: Vel = 0, Rot = -32.471, x = 2.034, y = 5.416, heading = -158.429, NCL = 0.953
    // Depot Position Start State: Vel = 0, Rot = 180, x = 0.644, y = 6.006, heading = -34.181, NCL 0.840

    // Centralized Field2d instances used by the robot for dashboard visualization.
    // These are static so other subsystems can update the field visualization without
    // requiring an AutoPlans instance reference during construction order.
    private static final Field2d s_mainField = new Field2d();
    private static final Field2d s_autoSelectorField = new Field2d();

    private PathPlannerAuto middleStartAuto, rightStartAuto;
    //private Command middleStartAuto;

    // Mapping of auto mode index -> Pose2d used by the auto-selector visualization.
    private static final Map<Integer, Pose2d> s_autoModePoses = new HashMap<>();

    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;

    /**
     * Construct AutoPlans and register any named PathPlanner commands that autos may call.
     * Accepts the IntakeSubsystem so intake commands can be created with the proper subsystem
     * instance.
     */
    public AutoPlans(IntakeSubsystem intake, AutoSelectorKnobSubsystem selector, ShooterSubsystem shooter) {
        // Publish the shared Field2d visualizations to SmartDashboard so they appear
        // in the simulator/dashboard. Doing this here keeps field-related configuration
        // in one place.
        SmartDashboard.putData("Field", s_mainField);
        SmartDashboard.putData("AutoSelectorField", s_autoSelectorField);

        // Initialize default auto-mode poses used by the selector visualization
        initializeAutoModePoses();

        // Set initial selector pose based on the current knob reading
        try {
            setAutoSelectorFieldRobotPose(getAutoModePose(selector.getAutoMode()));
        } catch (Throwable ignore) {
            // ignore if selector isn't ready
        }

        //Populate subsystems
        m_intake = intake;
        m_shooter = shooter;

        //Register commands
        registerCommands();

        //Create pathplanner autos before using them in autonomous mode
        createAutos();

    }

    /**Make all the Pathplanner autos */
    public void createAutos(){
        try{
            middleStartAuto = new PathPlannerAuto("Middle Start Auto");
            rightStartAuto = new PathPlannerAuto("Right Start Auto");
            
        } catch (Throwable t){
            System.out.println("Failed to create autos.");
        }
    }

    /** Register PathPlanner named commands for intake actions. */
    public void registerCommands(){
        try {
            Map<String, Command> eventMap = new HashMap<>();

            eventMap.put("IntakeUp", new IntakeUpCommand(m_intake));
            eventMap.put("IntakeDown", new IntakeDownCommand(m_intake));
            eventMap.put("IntakeOn", new RunIntakeCommandForward(m_intake));
            eventMap.put("IntakeReverse", new RunIntakeCommandReversed(m_intake));
            eventMap.put("IntakeOff", new RunIntakeCommandForward(m_intake).withTimeout(0));

            eventMap.put("StartShooterHub", new StartShooter(m_shooter,shooterPositions.HUB));
            eventMap.put("StartShooterTrench", new StartShooter(m_shooter,shooterPositions.TRENCH_CLOSE));
            eventMap.put("StartShooterDepot", new StartShooter(m_shooter,shooterPositions.DEPOT));
            eventMap.put("StartShooterOutpost", new StartShooter(m_shooter, shooterPositions.OUTPOST));

            eventMap.put("StopShooter", new StopShooter(m_shooter));

            eventMap.put("PrepareShooterHub", new PrepareShooterCommand(m_shooter,shooterPositions.HUB));
            eventMap.put("PrepareShooterTrench", new PrepareShooterCommand(m_shooter, shooterPositions.TRENCH_CLOSE));
            eventMap.put("PrepareShooterDepot", new PrepareShooterCommand(m_shooter, shooterPositions.DEPOT));
            eventMap.put("PrepareShooterOutpost", new PrepareShooterCommand(m_shooter, shooterPositions.OUTPOST));

            NamedCommands.registerCommands(eventMap);
        } catch (Throwable t) {
            System.out.println("Failed to register PathPlanner named commands: " + t.toString());
        }
    }

    /** Initialize the default auto-mode poses for the Auto Selector visualization. */
    public static void initializeAutoModePoses() {
        // Populate poses for auto modes 0..11 (same pattern used previously in AutoSelectorKnobSubsystem)
        for (int i = 0; i <= 11; i++) {
            double x = 1.0 + i * 0.5; // meters
            double y = 0.5; // meters
            double rotation = 0.0; // facing +X
            s_autoModePoses.put(i, new Pose2d(new edu.wpi.first.math.geometry.Translation2d(x, y), new edu.wpi.first.math.geometry.Rotation2d(rotation)));
        }
        // Set initial pose on the shared AutoSelectorField
        setAutoSelectorFieldRobotPose(s_autoModePoses.getOrDefault(0, new Pose2d()));
    }

    /** Return the stored Pose2d for a given auto mode index. */
    public static Pose2d getAutoModePose(int mode) {
        return s_autoModePoses.getOrDefault(mode, new Pose2d());
    }

    /** Update the AutoSelector field visualization for the given auto mode. */
    public static void updateAutoSelectorFieldForMode(int mode) {
        setAutoSelectorFieldRobotPose(getAutoModePose(mode));
    }

    /** Set the primary field robot pose (used by SwerveSubsystem). */
    public static void setMainFieldRobotPose(Pose2d pose) {
        try {
            s_mainField.setRobotPose(pose);
        } catch (Throwable ignore) {
            // ignore to avoid spamming logs if dashboard isn't available
        }
    }

    /** Set the auto-selector field robot pose (used by AutoSelectorKnobSubsystem). */
    public static void setAutoSelectorFieldRobotPose(Pose2d pose) {
        try {
            s_autoSelectorField.setRobotPose(pose);
        } catch (Throwable ignore) {
            // ignore to avoid spamming logs if dashboard isn't available
        }
    }

    //Returns a command based on the auto number given
    public Command getAutonomousCommand(int commandIndex){
        switch(commandIndex){
            default:
                return null;
            case 0:
                return null; //Do nothing
            case 1:
                return middleStartAuto; 
            case 2:
                return rightStartAuto; 
            case 3:
                return null; 
            case 4:
                return null;
            case 5:
                return null;
            case 6:
                return null;
            case 7:
                return null;
        }
    }
    
}
