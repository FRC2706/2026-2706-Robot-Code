// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
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

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final AutoSelectorKnobSubsystem m_autoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();
  private final PhotonSubsystem m_photonSubsystem = new PhotonSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_photonSubsystem);
  
  // Pathplanner testing
  private final AutoPlans m_autoPlans;
  private final SendableChooser<Command> autoChooser;

    // Controller
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
  m_autoPlans = new AutoPlans(m_intakeSubsystem, m_autoSelectorKnobSubsystem, m_shooterSubsystem);
  //autoChooser = AutoBuilder.buildAutoChooser(m_autoPlans.getAutonomousCommand(m_autoSelectorKnobSubsystem.getAutoMode()));
  autoChooser = AutoBuilder.buildAutoChooser("Drive Forward");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Zero the gyro such that forward is where the robot is currently looking
    driverController.start().onTrue(new ResetGyroCommand(m_swerveSubsystem));

    // Toggle intake ON/OFF
    JoystickButton intakeToggleButton =
        new JoystickButton(driverController.getHID(), XboxController.Button.kA.value);

    // Run intake in reverse while held
    JoystickButton reverseButton =
        new JoystickButton(driverController.getHID(), XboxController.Button.kB.value);

    // Toggle intake to go down
    JoystickButton intakeDownButton =
        new JoystickButton(driverController.getHID(), XboxController.Button.kX.value);

    // Toggle intake to go up
    JoystickButton intakeUpButton =
        new JoystickButton(driverController.getHID(), XboxController.Button.kY.value);

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
  public Command getAutonomousCommand() {
    // Return the command selected on the SendableChooser (built by AutoBuilder).
    Command selected = autoChooser.getSelected();

    if (selected != null) {
      return selected;
    }
    return null;
  }
}


  



