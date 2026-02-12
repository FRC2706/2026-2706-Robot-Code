package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

public class ShooterStateMachine {

    private boolean isRPMInRange = false;
    //private static double distanceFromHub  = 0.0;

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
 //Changes state based on Range and Mode
  public void updateState(){
    switch (desiredMode){
        case STOP_SHOOTER: // is the button pressed down
            currentState= States.IN_IDLE;
            break;
        case SHOOT: // button is pressed down
            if (isRPMInRange ==false){
                currentState = States.SPINNING_UP;
            } 
            else if (isRPMInRange) {
                currentState = States.READY;
            }
            break;
        default: 
            break;
    }
  }
    
}

// in_idle: switch to spinning_up when the button is held down
// spinning_up: switch to ready when the rpm is reached (isRPMInRange==true)
// switch back to in_idle as soon as button is no longer held down