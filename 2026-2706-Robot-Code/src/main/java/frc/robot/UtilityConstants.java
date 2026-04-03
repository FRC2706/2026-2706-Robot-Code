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
public final class UtilityConstants {
  public static final boolean debugMode = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;    
    public static final int kOperatorControllerPort = 1;
  }
   public static class RobotConstants {
    public static final int kSelectorSwitchPort = 0;

    public static final int kRPMConversionFactor = 1 / 3;

    public static final int kIntakeMotorID = 26;
    public static final int kIntakeUpDownMotorID = 25;
    public static final double kIntakeSpeed = 4000; 
    //Time before robot arm moves up
    public static final double kAgitateArmTimeMid = 1;
    //Time before robot arm moves down
    public static final double kAgitateArmTimeDown = 3.5;
    public static final double kAgitateRollerTime = 2;
    public static final double kReverseIntakeSpeed = 1000;
    public static final double kStowPosition = 65;
    public static final double kFloorPosition = 0;
    public static final double kMidPosition = 15;
    public static final double kTolerance = 0.3;
 
    public static final double kUpDownP = 0.012;
    public static final double kUpDownI = 0.0;
    public static final double kUpDownD = 0.0; 

    public static final double kSpeedP = 0.00008;
    public static final double kSpeedI = 0.0;
    public static final double kSpeedD = 0.005; 
    public static final double kSpeedV = 0.0022;
  }

  public static final class CANID {

    public static final int SHOOTER1 = 37;
    public static final int SHOOTER2 = 38;
    public static final int FEEDER = 39;
    public static final int INDEXER = 24;

  }

  public static final class shooterConstants {

    public static final byte MOTOR1_ID = CANID.SHOOTER1;
    public static final byte MOTOR2_ID = CANID.SHOOTER2;
    public static final byte FEEDER_MOTOR_ID = CANID.FEEDER;
    public static final byte INDEXER_MOTOR_ID = CANID.INDEXER;

    // PID VALUES

    public static final double shooterkP = 0.000325,
                               shooterkI = 0,
                               shooterkD = 0.015,
                               shooterkFF = 0.0023,

                               shooterAgressivekP = 0.000325,
                               shooterAgressivekI = 0,
                               shooterAgressivekD = 0.005,
                               shooterAgressivekFF=0.0023,

                               feederkP = 0.0001,
                               feederkI = 0.00003,
                               feederkD = 0.0001,
                               feederkFF = 0.001,

                               indexerkP = 0.000001,
                               indexerkI = 0,
                               indexerkD = 0,
                               indexerkFF = 0.00202;

    // Positions
    public static final class shooterPositions{
      public static final byte HUB = 0;
      public static final byte TRENCH_FAR = 1;
      public static final byte DEPOT = 2;
      public static final byte TRENCH_CLOSE = 7;
      public static final byte OUTPOST = 8;
    }

  }
  
  public static class VisionConstants {
    // Goal distance from the AprilTag (shooter distance to the net) in meters
    public static final double kGoalDistance = 1.5; // Adjust this value for optimal shooting distance
  }     
}