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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final AutoSelectorKnobSubsystem m_AutoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /** Read the knob voltage and convert it into an autonomous mode number (0-11). */
  private int getAutoMode() {
    double voltage = m_AutoSelectorKnobSubsystem.getVoltage(); // 0-5V
    System.out.println(voltage);
    if (voltage <= 2.64) {
      return 0;
    }
    else if (voltage <= 3.01){
      return 1;
    }
    else if (voltage <= 3.27){
      return 2;
    }
    else if (voltage <= 3.62){
      return 3;
    }
    else if (voltage <= 3.86){
      return 4;
    }
    else if (voltage <= 4.01){
      return 5;
    }
    else if (voltage <= 4.13){
      return 6;
    }
    else if (voltage <= 4.22){
      return 7;
    }
    else if (voltage <= 4.29){
      return 8;
    }
    else if (voltage <= 4.38){
      return 9;
    }
    else if (voltage <= 4.45){
      return 10;
    }
    else if (voltage <= 4.60){
      return 11;
    }
    else{
      return 0;
    }
    }
  


  /** This function returns the autonomous command based on the knob position. */
  public Command getAutonomousCommand() {
    int mode = getAutoMode();
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

