package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AutoSelectorKnobSubsystem extends SubsystemBase {

    private final AnalogInput m_knob = new AnalogInput(Constants.kSelectorSwitchPort);

    public AutoSelectorKnobSubsystem() {

    }

    public double getVoltage() {
        return m_knob.getVoltage();
    }

    /**
     * Read the knob voltage and convert it into an autonomous mode number (0-11).
     */
    public int getAutoMode() {
        double voltage = getVoltage(); // 0-5V
        System.out.println(voltage);
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
