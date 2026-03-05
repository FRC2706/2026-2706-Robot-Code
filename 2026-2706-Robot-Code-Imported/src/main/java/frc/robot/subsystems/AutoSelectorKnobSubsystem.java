package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;

public class AutoSelectorKnobSubsystem extends SubsystemBase {

    private final AnalogInput m_knob = new AnalogInput(Constants.kSelectorSwitchPort);

    // Field2d visualization for 2D field capability
    private final Field2d m_field = new Field2d();

    // Map of autonomous mode -> Pose2d for visualization on the field
    private final Map<Integer, Pose2d> m_autoModePoses = new HashMap<>();

    public AutoSelectorKnobSubsystem() {
        // Publish the Field2d to SmartDashboard so it appears in the simulator/dashboard
        SmartDashboard.putData("Field", m_field);

        // Example poses for each auto mode (0-11).
        // TODO: Replace these with the real positions/rotations for each autonomous routine.
        // Coordinates are in meters; rotation in radians.
        for (int i = 0; i <= 11; i++) {
            // lay out example positions along the X axis with small offset on Y
            double x = 1.0 + i * 0.5; // meters
            double y = 0.5; // meters
            double rotation = 0.0; // facing +X
            m_autoModePoses.put(i, new Pose2d(new Translation2d(x, y), new Rotation2d(rotation)));
        }
        // Set initial robot pose
        m_field.setRobotPose(m_autoModePoses.getOrDefault(getAutoMode(), new Pose2d()));
    }

    /**
     * Set the field robot pose explicitly.
     */
    public void setFieldPose(Pose2d pose) {
        m_field.setRobotPose(pose);
    }

    /**
     * Get the current robot pose being shown on the field.
     */
    public Pose2d getFieldPose() {
        // Field2d does not expose a getter for robot pose; return the stored mapping for the
        // current auto mode as the best-effort value.
        return m_autoModePoses.getOrDefault(getAutoMode(), new Pose2d());
    }

    /**
     * Update the field visualization to reflect the currently selected auto mode.
     */
    public void updateFieldForSelectedAuto() {
        int mode = getAutoMode();
        Pose2d pose = m_autoModePoses.getOrDefault(mode, new Pose2d());
        m_field.setRobotPose(pose);
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