// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.UtilityConstants.OperatorConstants;
import frc.robot.UtilityConstants.shooterConstants.shooterPositions;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.AlignShooterCommand;
import frc.robot.commands.ClearIndexerCommand;
import frc.robot.commands.IntakeAgitateCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.LockPoseCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShooter;

import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.commands.StopIndexerCommand;


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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final AutoSelectorKnobSubsystem m_autoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();

  // Pathplanner 
  private final AutoPlans m_autoPlans;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Left joystick controls the robot's translation movements (up moves the robot up, left moves the robot left, e.t.c)
    // Right joystick controls the rate of rotation (left rotates the robot counter clock-wise, right rotates the robot clock-wise)
    //If driving on red alliance, the controls should be flipped
    if (m_swerveSubsystem.isRedAlliance()){
      m_swerveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(
            m_swerveSubsystem,
            () -> driverController.getLeftY(), // Forward/backward
            () -> driverController.getLeftX(), // Left/right
            () -> -driverController.getRightX(),0.1,0.1)
      );
    }
    else{
      m_swerveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(
            m_swerveSubsystem,
            () -> -driverController.getLeftY(), // Forward/backward
            () -> -driverController.getLeftX(), // Left/right
            () -> -driverController.getRightX(),0.1,0.1)
      );
    }

    // Configure PathPlanner/AutoBuilder now that the swerve subsystem exists
    // This will configure AutoBuilder using the subsystem-provided callbacks.
    m_swerveSubsystem.setupPathPlanner();

    // Now that AutoBuilder is configured create autos
    m_autoPlans = new AutoPlans(intakeSubsystem, m_autoSelectorKnobSubsystem, m_ShooterSubsystem);

    configureBindings();
    // CameraServer.startAutomaticCapture();
  }

  private void configureBindings() {
    //Zero the gyro such that forward is where the robot is currently looking
    driverController.start().onTrue(new ResetGyroCommand(m_swerveSubsystem));
    
    //Turn shooter on and off at different rpm's
    m_operatorController.x().whileTrue(new StartShooter(m_ShooterSubsystem, shooterPositions.TRENCH_FAR)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.a().whileTrue(new StartShooter(m_ShooterSubsystem, shooterPositions.NEUTRAL)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.y().whileTrue(new StartShooter(m_ShooterSubsystem, shooterPositions.HUB)).onFalse(new StopShooter(m_ShooterSubsystem)); 
  // When B is held: first align to the AprilTag, then start the shooter using photon distance
    m_operatorController.b().whileTrue(
        new StartShooter(m_ShooterSubsystem, 3)
    ).onFalse(new StopShooter(m_ShooterSubsystem));   
    
  driverController.b().whileTrue(new AlignShooterCommand(m_swerveSubsystem));
    
    //Turn only the indexer when pressed
    m_operatorController.start().whileTrue(new ClearIndexerCommand(m_ShooterSubsystem)).onFalse(new StopIndexerCommand(m_ShooterSubsystem));

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

    intakeDownButton.toggleOnTrue(new IntakeDownCommand(intakeSubsystem));
    intakeUpButton.toggleOnTrue(new IntakeUpCommand(intakeSubsystem));
    intakeToggleButton.toggleOnTrue(new RunIntakeCommandForward(intakeSubsystem));
    reverseButton.whileTrue(new RunIntakeCommandReversed(intakeSubsystem));

    //Moves the intake up and down while turning the intake Defaults to moving it down after finishing
    m_operatorController.back().onTrue(new IntakeAgitateCommand(intakeSubsystem)).onFalse(new IntakeUpCommand(intakeSubsystem));
    
    //Locks the position of the robot to prevent moving when pressing "A" on the driver controller
    driverController.leftBumper().onTrue(new LockPoseCommand(m_swerveSubsystem)).onFalse(new LockPoseCommand(m_swerveSubsystem).withTimeout(0));

    // Allow remote triggering of AlignShooterCommand via NetworkTables key:
    // Set /commands/AlignShooter/Request = true (for example from Advantage Scope) to schedule the command.
    Trigger ntAlignRequest = new Trigger(() -> NetworkTableInstance.getDefault().getTable("commands").getEntry("AlignShooter/Request").getBoolean(false));
    ntAlignRequest.onTrue(new AlignShooterCommand(m_swerveSubsystem));
  }
  
  /** This function returns the autonomous command based on the knob position. */
  public Command getAutonomousCommand() {

    //Get the auto based on autonomous selector switch
    Command auto = m_autoPlans.getAutonomousCommand(m_autoSelectorKnobSubsystem.getAutoMode());

    return auto;
  }
  
}
