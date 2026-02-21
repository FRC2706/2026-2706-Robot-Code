// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;    
    public static final int kOperatorControllerPort = 1;
  }
  public static class RobotConstants {
    public static final int kSelectorSwitchPort = 0;
  }

  public static final class CANID {

    public static final int SHOOTER1 = 31;
    public static final int SHOOTER2 = 15;
    public static final int FEEDER = 40;
    //public static final int INDEXER = 27;

  }

  public static final class shooterConstants {

    public static final byte MOTOR1_ID = CANID.SHOOTER1;
    public static final byte MOTOR2_ID = CANID.SHOOTER2;
    public static final byte FEEDER_MOTOR_ID = CANID.FEEDER;
    //public static final byte INDEXER_MOTOR_ID = CANID.INDEXER;

    // PID VALUES

    public static final double kP1 = 0,
                               kI1 = 0,
                               kD1 = 0,
                               kFF1 = 0,
                               kP2 = 0,
                               kI2 = 0,
                               kD2 = 0,
                               kFF2 = 0,
                               kP3 = 0,
                               kI3 = 0,
                               kD3 = 0,
                               kFF3 = 0;
  }



}
