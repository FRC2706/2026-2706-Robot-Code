package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class FMS_Subsystem extends SubsystemBase {
    // Setup for reading the FMS message. 
    private final Timer rumbleTimer = new Timer();
    private boolean hubCurrentState = false;
    private boolean hubCurrentStateRumble = false;
    private boolean isHubActiveUpdate = isHubActive();
    private double matchTime;
    private DoublePublisher matchTimePub;
    private BooleanPublisher isHubActivePub;
    private BooleanPublisher isDSConnectedPub;
    private BooleanPublisher isFMSConnectedPub;
    private BooleanPublisher isJoystickConnectedPub;
    private boolean rumbling = false;
    private CommandXboxController driver;
    private CommandXboxController operator;

    public FMS_Subsystem(CommandXboxController driver, CommandXboxController operator) {
        // Initialize controllers
        this.driver = driver;
        this.operator = operator;
        // Dashboard setup includes Match time, hub status, and connection status of the driver station, FMS, and joysticks.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("FMSInfo");
        matchTimePub = table.getDoubleTopic("MatchTime").publish();
        isHubActivePub = table.getBooleanTopic("IsHubActive").publish();
        isDSConnectedPub = table.getBooleanTopic("IsConnected").publish();
        isFMSConnectedPub = table.getBooleanTopic("IsFMSConnected").publish();
        isJoystickConnectedPub = table.getBooleanTopic("IsJoystickConnected").publish();
    }
    
    @Override
    public void periodic(){
      try{
        //more dashboard setup stuff
        matchTime = DriverStation.getMatchTime();
        matchTimePub.set(matchTime);
        isHubActivePub.set(isHubActive());
        isDSConnectedPub.set(DriverStation.isDSAttached());
        isFMSConnectedPub.set(DriverStation.isFMSAttached());
        isJoystickConnectedPub.set(DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1));

        // Rumble section
        boolean isHubActiveUpdateRumble = isHubActive10Seconds();
        // This tells us when our hub status changes, and what it changes to.
        if (isHubActiveUpdateRumble != hubCurrentStateRumble) {

          // Controller rumble 
          driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
          operator.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
          rumbleTimer.reset(); rumbleTimer.start(); rumbling = true;
        }
        // Prevents constant controller rumble
        if (rumbling && rumbleTimer.hasElapsed(0.8)) {
          driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
          operator.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
          rumbling = false;
          rumbleTimer.stop();
        }
        // Prevents constant stream of print commands.
        hubCurrentStateRumble = isHubActiveUpdateRumble;
      }
      catch (Exception e){
        
      }
    }
    // Boolean that updates the hub status.
    public boolean isHubActive() {
      Optional<Alliance> alliance = DriverStation.getAlliance();

      // If we have no alliance, we cannot be enabled, therefore no hub.
      if (alliance.isEmpty()) {
        return false;
      }
      // Hub is always enabled in autonomous.
      if (DriverStation.isAutonomousEnabled()) {
        return true;
      }
      // At this point, if we're not teleop enabled, there is no hub.
      if (!DriverStation.isTeleopEnabled()) {
        return false;
      }

      // We're teleop enabled, compute.
        matchTime = DriverStation.getMatchTime();
      String gameData = DriverStation.getGameSpecificMessage();
      // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
      if (gameData.isEmpty()) {
        return true;
      }
      boolean redInactiveFirst = false;
      switch (gameData.charAt(0)) {
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> {
          // If we have invalid game data, assume hub is active.
          return true;
        }
      }

      // Shift was is active for blue if red won auto, or red if blue won auto.
      boolean shift1Active = switch (alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
      };

      // Hub is always enabled in autonomous.
      if (DriverStation.isAutonomousEnabled()) {
        return true;
      }

      if (matchTime > 130) {
        // Transition shift, hub is active.
        return true;
      } else if (matchTime > 105) {
        // Shift 1
        return shift1Active;
      } else if (matchTime > 80) {
        // Shift 2
        return !shift1Active;
      } else if (matchTime > 55) {
        // Shift 3
        return shift1Active;
      } else if (matchTime > 30) {
        // Shift 4
        return !shift1Active;
      } else {
        // End game, hub always active.
        return true;
      }
      
    }
    
    // Boolean that updates 10 seconds before hub changes to trigger the rumble.
    public boolean isHubActive10Seconds() {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime < 115 && matchTime > 114.8) {
        // 10 seconds before shift 2
        return true;
      } else if (matchTime < 90 && matchTime > 89.8) {
        // 10 seconds before shift 3
        return true;
      } else if (matchTime < 65 && matchTime > 64.8) {
        // 10 seconds before shift 4
        return true;
      } else if (matchTime < 40 && matchTime > 39.8) {
        // 10 seconds before end game
        return true;
      } else {
        return false;
      }
    }
}
