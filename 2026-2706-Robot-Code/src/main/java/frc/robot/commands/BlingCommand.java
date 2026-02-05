// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlingSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class BlingCommand extends InstantCommand {
  public enum BlingColour {
    DISABLED,
    PURPLE,
    BLUE,
    RED,
    YELLOW,
    HONEYDEW,
    RAINBOW,
    FIRE,
    RGBFADE,
    WHITESTROBE,
    REDSTROBE,
    YELLOWSTROBE,
    BLUESTROBE,
    PURPLESTROBE,
  }
  public BlingSubsystem bling = BlingSubsystem.getINSTANCE();
  public BlingColour blingColour;

  public BlingCommand(BlingColour colour) {
    blingColour = colour;

    // Use addRequirements() here to declare subsystem dependencies.
    if( bling != null )
      addRequirements(bling);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (bling != null)
    {
      if (blingColour != null) {
        bling.setBrightness(-1);
      }
      switch( blingColour )
      {
        case DISABLED:
          bling.setDisabled();
          break;
        case PURPLE:
          bling.setPurple();
          break;
        case BLUE:
          bling.setBlue();
          break;
        case RED:
          bling.setRed();
          break;
        case YELLOW:
          bling.setYellow();
          break;
        case HONEYDEW:
          bling.setHoneydew();
          break;
        case RAINBOW: 
          bling.setRainbow();
          break;
        case FIRE: 
          bling.setFire();
          break;
        case RGBFADE:
          bling.setRgbFade();
          break;
        case WHITESTROBE:
          bling.setWhiteStrobe();
          break;
        case REDSTROBE:
          bling.setRedStrobe();
          break;
        case YELLOWSTROBE:
          bling.setYellowStrobe();
          break;
        case BLUESTROBE:
          bling.setBlueStrobe();
          break;
        case PURPLESTROBE:
          bling.setPurpleStrobe();
          break;
        default:
          break;
      }
    }

    }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
