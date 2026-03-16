// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.UtilityConstants.OperatorConstants;
import frc.robot.UtilityConstants.shooterConstants.shooterPositions;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShooter;

import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.pathplanner.lib.auto.AutoBuilder;
import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

// Pathplanner testing
import frc.robot.subsystems.AutoPlans;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystem
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final AutoSelectorKnobSubsystem m_autoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();
  // Pathplanner testing
  private final AutoPlans m_autoPlans;
  private final SendableChooser<Command> autoChooser;

    // Controller
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

  // Silence repeated joystick connection warnings in the simulator; set to false to re-enable
  // warnings on a real driver station if you want to be notified when controllers unplug.
  DriverStation.silenceJoystickConnectionWarning(true);
  SmartDashboard.putString("ControllerStatus", "If you see joystick warnings, verify controller is plugged in and port index (Constants.OperatorConstants.kDriverControllerPort)");

    m_swerveSubsystem.setDefaultCommand(
       new SwerveDriveCommand(
          m_swerveSubsystem,
          () -> -m_driverController.getLeftY(), // Forward/backward
          () -> -m_driverController.getLeftX(), // Left/right
          () -> -m_driverController.getRightX(),0.1,0.1)
    );

    // Configure PathPlanner/AutoBuilder now that the swerve subsystem exists
    // This will configure AutoBuilder using the subsystem-provided callbacks.
    m_swerveSubsystem.setupPathPlanner();

  // PathPlanner named commands for intake are registered by AutoPlans

  // Now that AutoBuilder is configured, create autos and the chooser
  m_autoPlans = new AutoPlans(m_intakeSubsystem, m_autoSelectorKnobSubsystem, m_ShooterSubsystem);
  autoChooser = AutoBuilder.buildAutoChooser(m_autoPlans.getAutonomousCommand(m_autoSelectorKnobSubsystem.getAutoMode()));
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();

    CameraServer.startAutomaticCapture();
  }

  private void configureBindings() {


    //Zero the gyro such that forward is where the robot is currently looking
    driverController.start().onTrue(new ResetGyroCommand(m_swerveSubsystem));
    
    m_operatorController.x().whileTrue(new StartShooter(m_ShooterSubsystem, 1)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.a().whileTrue(new StartShooter(m_ShooterSubsystem, 2)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.y().whileTrue(new StartShooter(m_ShooterSubsystem, 0)).onFalse(new StopShooter(m_ShooterSubsystem)); 

    // Toggle intake ON/OFF
    JoystickButton intakeToggleButton =
        new JoystickButton(m_operatorController.getHID(), XboxController.Button.kRightBumper.value);

    // Run intake in reverse while held
    JoystickButton reverseButton =
        new JoystickButton(m_operatorController.getHID(), XboxController.Button.kLeftBumper.value);

    // Toggle intake to go down
    Trigger intakeDownButton =
        new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.1);

    // Toggle intake to go up
    Trigger intakeUpButton =
        new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.1);

    intakeDownButton.toggleOnTrue(new IntakeDownCommand(m_intakeSubsystem));
    intakeUpButton.toggleOnTrue(new IntakeUpCommand(m_intakeSubsystem));
    intakeToggleButton.toggleOnTrue(new RunIntakeCommandForward(m_intakeSubsystem));
    reverseButton.whileTrue(new RunIntakeCommandReversed(m_intakeSubsystem));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  /** This function returns the autonomous command based on the knob position. */
  public Command getAutonomousCommand() {

    // Return the command selected on the SendableChooser (built by AutoBuilder).
    Command selected = autoChooser.getSelected();
    if (selected != null) {
      return selected;
    }
    //Manually create and send the autos -- Happens if smart dashboard has an issue
    else{
      //Get the autoname based on autonomous selector switch
      String autoName = m_autoPlans.getAutonomousCommand(m_autoSelectorKnobSubsystem.getAutoMode());

      //Shooting only at hub is manually pased
      if (autoName == "Hub Shoot Only Auto"){
        return Commands.deadline(new StartShooter(m_ShooterSubsystem, shooterPositions.HUB), new WaitCommand(20));
      }
      //Shooting only at the right trench is manually passed
      else if (autoName == "Trench Shoot Only Auto"){
        return Commands.deadline(new StartShooter(m_ShooterSubsystem, shooterPositions.TRENCH_CLOSE), new WaitCommand(20));
      }
      //Check that the auto exists
      else if (autoName != null){
        try{
          return new PathPlannerAuto(autoName);
        }
        catch (Error e){
          //In the event the auto does not exist/there is a bug, return a "do nothing" auto
          return null;
        }
      }
      //Do nothing auto
      else{
        return null;
      }
    }
  }
}


  



