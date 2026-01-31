package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.Config;

public class BlingSubsystem extends SubsystemBase {
   
  private CANdle candle;
  public final double Brightness = 0.5;
  private static BlingSubsystem INSTANCE = null;

  /**
   * Creates a new Bling. */
   
  private BlingSubsystem() {
    if (Config.CANID.CANDLE != -1) {
      // SubsystemChecker.subsystemConstructed(SubsystemType.BlingSubsystem);
      
      candle = new CANdle(Config.CANID.CANDLE);
      
      CANdleConfiguration config = new CANdleConfiguration();
      config.LED.StripType = StripTypeValue.RGB; // set the strip type to RGB
      config.LED.BrightnessScalar = Brightness; // dim the LEDs to half brightness

      candle.getConfigurator().apply(config);
    } else {
      candle = null;
    }
  }

  public static BlingSubsystem getINSTANCE() {
    if (Config.CANID.CANDLE == -1) {
      INSTANCE = null;
    } else if (INSTANCE == null) {
      INSTANCE = new BlingSubsystem();
    }

    return INSTANCE;
  }


  public void setBrightness(double brightness) {
    var CandleConfigurator = candle.getConfigurator();
    var CandleConfiguration = new CANdleConfiguration();
    CandleConfigurator.refresh(CandleConfiguration);
    
    if (brightness < 0) {
      CandleConfiguration.LED.BrightnessScalar = Brightness;
    } else {
      CandleConfiguration.LED.BrightnessScalar = brightness;
    }

    candle.getConfigurator().apply(CandleConfiguration);
  }

  public void clearAnimation() {
    for (int x=0;x<8;x++) {
      candle.setControl(new EmptyAnimation(x)); //use setControl to set animations !!!!!!!!
    }
  }

  public void setLEDColour(int red, int green, int blue) {
    candle.setControl(new SolidColor(0, 7).withColor(new RGBWColor(red,green,blue)));
  }

  public void setDisabled() {
    setBrightness(0.0);
    clearAnimation();
  }

  public void setOrange() {
    clearAnimation();
    setLEDColour(245, 141, 66);
  }

  public void setPurple() {
    clearAnimation();
    setLEDColour(138, 43, 226);
  }

  public void setBlue() {
    clearAnimation();
    setLEDColour(0, 0, 255);
  }

  public void setRed() {
    clearAnimation();
    setLEDColour(255, 0, 0);
  }

  public void setHoneydew() {
    clearAnimation();
    setLEDColour(240, 255, 240);
  }

  public void setYellow() {
    clearAnimation();
    setLEDColour(255, 255, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  //public void setAnimation(Animation animation) {
  //  candle.animate(animation);
  //}
}
