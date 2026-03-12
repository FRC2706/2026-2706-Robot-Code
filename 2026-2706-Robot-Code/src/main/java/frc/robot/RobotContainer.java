// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AutoSelectorKnobSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopShooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AutoSelectorKnobSubsystem m_AutoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();


  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
        //.onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_operatorController.a().whileTrue(new StartShooter(m_ShooterSubsystem)).onFalse(new StopShooter(m_ShooterSubsystem));
  }

  /** This function returns the autonomous command based on the knob position. */
  public Command getAutonomousCommand() {
    int mode = m_AutoSelectorKnobSubsystem.getAutoMode();
    System.out.println("Auto Mode = " + mode); // debug print
    
    switch (mode) {
      case 0:
        return null; // do nothing
      case 1:
        return new PrintCommand("1");
        //DriveDistance(39, 0.3, m_robotDrive);
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
    }}}

