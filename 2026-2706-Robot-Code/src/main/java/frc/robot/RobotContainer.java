// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ResetGyroCommand;

import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.RunIntakeCommandForward;
import frc.robot.commands.RunIntakeCommandReversed;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AutoSelectorKnobSubsystem m_AutoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

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
  public Command getAutonomousCommand() {
    int mode = m_AutoSelectorKnobSubsystem.getAutoMode();
    System.out.println("Auto Mode = " + mode); // debug print
    
    switch (mode) {
      case 0:
        return null; // do nothing
      case 1:
        return null;
      case 2:
        return new PrintCommand("2");
        //DriveTimed(2.0, 0.3, m_robotDrive);
      case 3:
        return new PrintCommand("3");
        //null;
      case 4:
        return new PrintCommand("4");
        //null;
      case 5:
        return new PrintCommand("5");
        //null;
      case 6:
        return new PrintCommand("6");
        //null;
      case 7:
        return new PrintCommand("7");
        //null;
      case 8:
        return new PrintCommand("8");
        //null;
      case 9:
        return new PrintCommand("9");
        //null;
      case 10:
        return new PrintCommand("10");
        //null;
      case 11:
        return new PrintCommand("11");
        //null;
      default:
        return null;
    }
  }
}
