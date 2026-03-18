package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityConstants;

import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.geometry.Pose2d;


public class AutoSelectorKnobSubsystem extends SubsystemBase {

    private final AnalogInput m_knob = new AnalogInput(UtilityConstants.RobotConstants.kSelectorSwitchPort);

    //CANBus used for the candle
    private final CANBus m_CANBus = new CANBus("rio");

    //Instantiating CANdle
    private final CANdle m_CANdle = new CANdle(50, m_CANBus);

    //Purple colour for the CANdle
    private final RGBWColor m_PurpleColour = new RGBWColor(255, 0, 255);

    //No colour for CANdle
    private final RGBWColor m_BlackColour = new RGBWColor(0,0,0);

    //White colour for the CANdle
    private final RGBWColor m_WhiteColour = new RGBWColor(255,255,255);

    //Solid colour request for reseting the CANdle
    private final SolidColor m_ResetRequest = new SolidColor(0, 7).withColor(m_BlackColour);

    public AutoSelectorKnobSubsystem() {
        //Applying configurations to the CANdle
        CANdleConfiguration m_CaNdleConfiguration = new CANdleConfiguration();

        m_CaNdleConfiguration.LED.BrightnessScalar = 0.5;
        m_CaNdleConfiguration.LED.StripType = StripTypeValue.RGB;
        m_CaNdleConfiguration.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        m_CANdle.getConfigurator().apply(m_CaNdleConfiguration);
    }

    @Override
    public void periodic(){
        if (getAutoMode() == 0){
            //Light up nothing if the auto mode is 0
            resetCANdle();
        }
        else if (getAutoMode() <= 7){
            //Light up the same amount of led's as the selected auto's index 
           lightUpCandle(getAutoMode());
        }
    }

    //Reset all the led's on the CANdle
    public void resetCANdle(){
        m_CANdle.setControl(m_ResetRequest);
    }

    //Light up a certain amount of led's with purple. On overflow, the colour will change to white
    public void lightUpCandle(int ledAmount){
        //Reset the CANdle first
        resetCANdle();
        
        //Check if the ledAmount will result in an overflow or not
        if (ledAmount <= 8){
            m_CANdle.setControl(new SolidColor(0, ledAmount - 1).withColor(m_PurpleColour));
        }
        else{
            m_CANdle.setControl(new SolidColor(0, ledAmount % 8).withColor(m_WhiteColour));
        }
    }

    /**
     * Set the field robot pose explicitly.
     */
    public void setFieldPose(Pose2d pose) {
        frc.robot.subsystems.AutoPlans.setAutoSelectorFieldRobotPose(pose);
    }

    /**
     * Get the current robot pose being shown on the field.
     */
    public Pose2d getFieldPose() {
        // Delegates to AutoPlans which owns the auto-mode pose mapping.
        return frc.robot.subsystems.AutoPlans.getAutoModePose(getAutoMode());
    }

    /**
     * Update the field visualization to reflect the currently selected auto mode.
     */
    public void updateFieldForSelectedAuto() {
        int mode = getAutoMode();
        frc.robot.subsystems.AutoPlans.updateAutoSelectorFieldForMode(mode);
    }

    public double getVoltage() {
        return m_knob.getVoltage();
    }

    /**
     * Read the knob voltage and convert it into an autonomous mode number (0-11).
     */
    public int getAutoMode() {
        double voltage = getVoltage(); // 0-5V
        if (voltage <= 2.64) {
            return 0;
        } else if (voltage <= 3.01) {
            return 1;
        } else if (voltage <= 3.27) {
            return 2;
        } else if (voltage <= 3.62) {
            return 3;
        } else if (voltage <= 3.86) {
            return 4;
        } else if (voltage <= 4.01) {
            return 5;
        } else if (voltage <= 4.13) {
            return 6;
        } else if (voltage <= 4.22) {
            return 7;
        } else if (voltage <= 4.29) {
            return 8;
        } else if (voltage <= 4.38) {
            return 9;
        } else if (voltage <= 4.45) {
            return 10;
        } else if (voltage <= 4.60) {
            return 11;
        } else {
            return 0;
        }
    }
}