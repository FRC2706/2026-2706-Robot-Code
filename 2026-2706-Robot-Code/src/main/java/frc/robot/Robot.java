// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Map;
import java.util.Optional;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  
    initDashBoard();
  }

  void initDashBoard() {
    ShuffleboardTab dashboard = Shuffleboard.getTab("Driver Screen");

    // Match Time (Dial Widget)
    dashboard.addNumber("Remain Match Time", () -> getMatchTime())
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Min", 0, "Max", 140))
      .withPosition(1, 1)
      .withSize(2, 2);

    // Robot Mode (Text Widget)
    dashboard.addString("Robot Mode", () -> {
      if (isDisabled()) return "Disabled";
      if (isAutonomous()) return "Autonomous";
      if (isTeleop()) return "Teleop";
      return "Test";
    }).withPosition(1,3)
      .withSize(1, 1);

    // The Hub State Indicator (Massive Box)
    dashboard.addBoolean("My Hub", () -> isMyHubActive())
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withProperties(Map.of(
          "Color when true", "#00FF00",
          "Color when false", "#FF0000"
      ))
      .withPosition(4, 1)
      .withSize(1, 1);

    dashboard.addBoolean("Opponent Hub", () -> isOpponentHubActive())
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withProperties(Map.of(
          "Color when true", "#00FF00",
          "Color when false", "#FF0000"
      ))
      .withPosition(4, 2)
      .withSize(1, 1);

    dashboard.addNumber("Hub Switch Count Down", () -> getHubSwitchCountdown())
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Min", 0, "Max", 25))
      .withPosition(5, 1)
      .withSize(2, 2);
  }

  private double a = 140.0;
  public double getMatchTime() {
    // TEST
    return a;
    // return Timer.getMatchTime();
  }

  public boolean isMyHubActive() {
    return isHubActive(isActiveFirst());
  }

  public boolean isOpponentHubActive() {
    return isHubActive(!isActiveFirst());
  }

  public boolean isHubActive(boolean startedActive) {
    double time = getMatchTime();

    // AUTO: Always active
    if (DriverStation.isAutonomousEnabled()) return true;

    // THE SHIFT LOGIC
    if (time > 130.0) { // before 2:10
        return true;
    } else if (time > 105.0) { // 1:45 to 2:10
        return startedActive; // Swap 1
    } else if (time > 80.0) { // 1:20 to 1:45
        return !startedActive;  // Swap 2
    } else if (time > 55.0) { // 0:55 to 1:20
        return startedActive; // Swap 3
    } else if (time > 30.0)  { // 0:30 to 0:55
        return !startedActive;  // Swap 4
    } else {  // end game
      return true;
    }
  }

  public double getHubSwitchCountdown() {
    double time = getMatchTime();

    // If not in a match, or in endgame, no upcoming switch
    if (time < 0) return 0;
    if (DriverStation.isAutonomousEnabled()) return 0;
    if (time <= 30.0) return 0;

    double nextSwitch;

    if (time > 130.0) {
        nextSwitch = 130.0;
    } else if (time > 105.0) {
        nextSwitch = 105.0;
    } else if (time > 80.0) {
        nextSwitch = 80.0;
    } else if (time > 55.0) {
        nextSwitch = 55.0;
    } else {
        nextSwitch = 30.0;
    }

    return time - nextSwitch;
  }

  private boolean isActiveFirst() {
    if (DriverStation.isAutonomousEnabled()) return false;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) return false;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) { return false; }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return false;
      }
    }

    boolean inactiveFirst = switch (alliance.get()) {
      case Red -> redInactiveFirst;
      case Blue -> !redInactiveFirst;
    };

    return !inactiveFirst;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    a -= 20.0 / 1000.0;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
