package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor1;
  private SparkMax shooterMotor2;
  private SparkMax feederMotor; // CANNOT be faster than shooterMotor RPM
  private SparkMax indexerMotor;
  
  public ShooterSubsystem() {}

   private boolean isRPMInRange = false;
  /**
  
   * @return 
   */
  //Two different modes we are using
    private ShooterModes desiredMode = ShooterModes.STOP_SHOOTER;
    private States currentState = States.IN_IDLE;

    //Defining different modes and their parameters(RPM and Voltage)
    public static enum ShooterModes {
        STOP_SHOOTER(0, 0),
        SHOOT(0,0);

        double v, RPM;

        private ShooterModes (double voltage, double velo){
            v = voltage;
            RPM = velo;
        }

        public double getDesiredVoltage(){
            return v;
        }

        public double getDesiredSpeedRPM () {
            return RPM;
        }
  }

  //Defining states we are using
  public static enum States {
    IN_IDLE, // shooter, feeder, indexer off
    SPINNING_UP, // shooter spinning to desired rpm, feeder, indexer off
    READY // everything on
  }
  //Setting desired mode and parameters
  public void setMode (ShooterModes desiredMode){
    this.desiredMode = desiredMode;

  }

  public double getDesiredVoltage(){
    return desiredMode.getDesiredVoltage();
     // Use calculated RPM to set voltage used by motors
  }

  public double getDesiredVelocityRPM (){
    return desiredMode.getDesiredSpeedRPM();
    //Goal: Get average shooting distance from hardware and set an average RPM
    //Reach goal: Use data provided by vision snensors (distance from hub) to calculate speed needed
  }

  // Logging the needed values
  public ShooterModes getDesiredMode(){
        return desiredMode;
  }
  //Returns current state
  public States getCurrentState(){
    return currentState;
  }

  //Sets the variable inRange to know weather the RPM is correct for shooting or not
  public void checkShooterRPM(BooleanSupplier toRun){
    if (toRun != null)
        isRPMInRange = toRun.getAsBoolean();
    else 
        isRPMInRange = false;
  }


  public void stop(){
        System.out.println("stop cmd called");
        shooterMotor1.stopMotor(); 
        shooterMotor2.stopMotor(); 
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
  }

  public void spinningUp(){
        System.out.println("stop cmd called");
        shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
  }

  public void ready(){
        System.out.println("stop cmd called");
        shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage

        // divide by 2 is placeholder but need to figure out difference in voltage for feeder+indexer vs shooter motors
        feederMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2); 
        indexerMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2);
  }



  @Override
  public void periodic() {
        switch (currentState){
        case IN_IDLE: // is the button pressed down
            stop();
            break;
        case SPINNING_UP: // button is pressed down
            spinningUp();
            break;
        case READY:
            ready();
            break;
        default: 
            break;
    }
  }

  @Override
  public void simulationPeriodic() {
    
  }
}