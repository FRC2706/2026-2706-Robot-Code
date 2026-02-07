package frc.robot.subsystems;

public class ShooterStateMachine {

    private boolean isInRange = false;
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
    IN_IDLE,
    SPINING_UP,
    READY
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
  public void isRPMInRange(BooleanSupplier toRun){
    if (toRun != null)
        isInRange = toRun.getAsBoolean();
    else 
        isInRange = false;
  }
 //Changes state based on Range and Mode
  public void updateState(){
    switch (desiredMode){
        case STOP_SHOOTER:
            currentState= States.IN_IDLE;
            break;
        case SHOOT:
            if (isInRange ==false){
                currentState = States.SPINING_UP;
            } 
            else if (isInRange) {
                currentState = States.READY;
            }
            break;
        default: 
            break;
    }
  }
    
}