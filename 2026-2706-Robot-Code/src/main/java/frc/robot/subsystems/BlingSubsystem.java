package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;

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

  // speed in phoenix5 was from 0-1 
  // framerate in phoenix6 is from 0-1000
  
  public void setRainbow() {
     //* @param brightness The brightness of the LEDs 1
     //* @param speed How fast the rainbow travels through the leds 0.01
     //* @param numLed How many LEDs are controlled by the CANdle 64
    RainbowAnimation rainbowAnim = new RainbowAnimation(0, 7).withBrightness(1).withFrameRate(10);

    candle.setControl(rainbowAnim);
  }

  public void setFire()
  {
     //* @param brightness How bright should the animation be 1
     //* @param speed How fast will the flame be processed at 0.000001
     //* @param numLed How many LEDs is the CANdle controlling 64
     //* @param sparking The rate at which the Fire "Sparks" 0.8
     //* @param cooling The rate at which the Fire "Cools" along the travel 0.4
    FireAnimation fireAnimation = new FireAnimation(0,7).withBrightness(1).withFrameRate(0.001).withSparking(0.8).withCooling(0.4);

    candle.setControl(fireAnimation);
  }

  public void setWhiteStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 255
   //  * @param b How much blue should the color have 255
   //  * @param w How much white should the color have 255
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0, 7).withColor(new RGBWColor(255, 255, 255, 255)).withFrameRate(800);

    candle.setControl(strobeAnimation);
  }

  public void setPurpleStrobe()
  {
   //  * @param r How much red should the color have 138
   //  * @param g How much green should the color have 43
   //  * @param b How much blue should the color have 226
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0,7).withColor(new RGBWColor(138, 43, 226, 127)).withFrameRate(1);

    candle.setControl(strobeAnimation);
  }

  public void setRedStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 0
   //  * @param b How much blue should the color have 0
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0,7).withColor(new RGBWColor(255, 0, 0, 127)).withFrameRate(1);

    candle.setControl(strobeAnimation);
  }

  public void setBlueStrobe()
  {
   //  * @param r How much red should the color have 0
   //  * @param g How much green should the color have 0
   //  * @param b How much blue should the color have 255
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0,7).withColor(new RGBWColor(0, 0, 255, 127)).withFrameRate(1);

    candle.setControl(strobeAnimation);
  }

  public void setYellowStrobe()
  {
   //  * @param r How much red should the color have 255
   //  * @param g How much green should the color have 255
   //  * @param b How much blue should the color have 0
   //  * @param w How much white should the color have 127
   //  * @param speed How fast should the color travel the strip 0.001
   //  * @param numLed How many LEDs the CANdle controls 64
    StrobeAnimation strobeAnimation = new StrobeAnimation(0,7).withColor(new RGBWColor(255, 255, 0, 127)).withFrameRate(1);

    candle.setControl(strobeAnimation);
  }

  public void setRgbFade()
  {
     //* @param brightness How bright the LEDs are 0.7
     //* @param speed How fast the LEDs fade between Red, Green, and Blue 0.1
     //* @param numLed How many LEDs are controlled by the CANdle 64
    RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(0,7).withBrightness(0.7).withFrameRate(100);

    candle.setControl(rgbFadeAnimation);
  }
}
