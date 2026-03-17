// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.UtilityConstants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ResetGyroCommand;

import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AutoSelectorKnobSubsystem m_AutoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Left joystick controls the robot's translation movements (up moves the robot up, left moves the robot left, e.t.c)
    // Right joystick controls the rate of rotation (left rotates the robot counter clock-wise, right rotates the robot clock-wise)
    m_swerveSubsystem.setDefaultCommand(
      new SwerveDriveCommand(m_swerveSubsystem,
       () -> -driverController.getLeftY(), 
       () -> -driverController.getLeftX(), 
       () -> -driverController.getRightX(),
       0.1,
       0.1
       )
    );
    // Configure the trigger bindings
    configureBindings();
    //CameraServer.startAutomaticCapture();
  }

  private void configureBindings() {
    //Zero the gyro such that forward is where the robot is currently looking
    driverController.start().onTrue(new ResetGyroCommand(m_swerveSubsystem));
    
    m_operatorController.x().whileTrue(new StartShooter(m_ShooterSubsystem, 1)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.a().whileTrue(new StartShooter(m_ShooterSubsystem, 2)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.y().whileTrue(new StartShooter(m_ShooterSubsystem, 0)).onFalse(new StopShooter(m_ShooterSubsystem)); 
    m_operatorController.b().whileTrue(new StartShooter(m_ShooterSubsystem, 3)).onFalse(new StopShooter(m_ShooterSubsystem));
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /** This function returns the autonomous command based on the knob position. */
  public Command getAutonomousCommand() {
    int mode = m_AutoSelectorKnobSubsystem.getAutoMode();    
    switch (mode) {
      case 0:
        return null; // do nothing
      case 1:
        return null;
      case 2:
        return new PrintCommand("2");
      case 3:
        return new PrintCommand("3");
      case 4:
        return new PrintCommand("4");
      case 5:
        return new PrintCommand("5");
      case 6:
        return new PrintCommand("6");
      case 7:
        return new PrintCommand("7");
      case 8:
        return new PrintCommand("8");
      case 9:
        return new PrintCommand("9");
      case 10:
        return new PrintCommand("10");
      case 11:
        return new PrintCommand("11");
      default:
        return null;
    }
  }
}