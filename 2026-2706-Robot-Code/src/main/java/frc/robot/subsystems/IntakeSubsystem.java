// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.ResourceBundle.Control;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkSoftLimit.SoftLimitDirection;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.UtilityConstants;
import frc.robot.UtilityConstants.*;

import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * The Intake class represents the intake subsystem of the robot.
 * It controls the intake motor and provides methods to set the motor power,
 * stop the motor, and get the motor's RPM.
 */
public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor;
    private SparkMax intakeUpDownMotor;
    private final RelativeEncoder intakeUpDownEncoder;
    private final SparkClosedLoopController intakeUpDownPID;
    private final SparkClosedLoopController m_intakeController; 

    // Cached PID values to detect changes from dashboard
    private double lastP = RobotConstants.kUpDownP;
    private double lastI = RobotConstants.kUpDownI;
    private double lastD = RobotConstants.kUpDownD;

    /**
     * Constructs a new Intake subsystem.
     * Initializes the intake motor.
     */
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(UtilityConstants.RobotConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeController = intakeMotor.getClosedLoopController();

        intakeUpDownMotor = new SparkMax(UtilityConstants.RobotConstants.kIntakeUpDownMotorID, MotorType.kBrushless);
        intakeUpDownEncoder = intakeUpDownMotor.getEncoder();
        intakeUpDownPID = intakeUpDownMotor.getClosedLoopController();
        
        // Resets the position to 0, turn the robot off while in up position to prevent things breaking.
        intakeUpDownEncoder.setPosition(0.0);

        SparkMaxConfig upDownConfig = new SparkMaxConfig();
            upDownConfig.closedLoop
                .p(RobotConstants.kUpDownP)
                .i(RobotConstants.kUpDownI)
                .d(RobotConstants.kUpDownD);

            upDownConfig.softLimit
                .forwardSoftLimit(RobotConstants.kStowPosition)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(RobotConstants.kFloorPosition)
                .reverseSoftLimitEnabled(true);

            upDownConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
            intakeUpDownMotor.configure(upDownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            

            SparkMaxConfig speedConfig = new SparkMaxConfig();
            speedConfig.closedLoop
                .p(RobotConstants.kSpeedP)
                .i(RobotConstants.kSpeedI)
                .d(RobotConstants.kSpeedD);

            speedConfig.closedLoop.feedForward.kV(RobotConstants.kSpeedV);

            intakeMotor.configure(speedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Pre-populate dashboard tuning fields with current constants
        SmartDashboard.putNumber("UpDown/P Gain", RobotConstants.kUpDownP);
        SmartDashboard.putNumber("UpDown/I Gain", RobotConstants.kUpDownI);
        SmartDashboard.putNumber("UpDown/D Gain", RobotConstants.kUpDownD);
        SmartDashboard.putNumber("Intake/Speed", RobotConstants.kIntakeSpeed);
    }

    /**
     * Starts the intake motor.
     */
    public void startIntake() {
        m_intakeController.setSetpoint(RobotConstants.kIntakeSpeed, ControlType.kVelocity);

    }

    /**
     * Starts the intake motor, but backwards.
     */
    public void reverseIntake() {
        m_intakeController.setSetpoint(-RobotConstants.kReverseIntakeSpeed, ControlType.kVelocity);

    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake(){
        
        intakeMotor.set(0);

    }

    /**
     * Gets the raw RPM of the intake motor.
     * 
     * @return The raw RPM of the intake motor.
     */
    public double getRawMotorRPM(){

        return intakeMotor.getEncoder().getVelocity();

    }

    /**
     * Moves the intake down.
     */
    public void moveIntakeDown() {
    intakeUpDownPID.setSetpoint(RobotConstants.kFloorPosition, ControlType.kPosition);
}

    /**
     * Moves the intake up.
     */
    public void moveIntakeUp() {
    intakeUpDownPID.setSetpoint(RobotConstants.kStowPosition, ControlType.kPosition);
}

    /**
     * Periodically updates the SmartDashboard with the intake motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            SmartDashboard.putNumber("Intake RPM", getRawMotorRPM());
            SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());

            // Live PID telemetry
            SmartDashboard.putNumber("UpDown/Current Position", intakeUpDownEncoder.getPosition());

            // Check if gains were changed on the dashboard and apply them live
            double p = SmartDashboard.getNumber("UpDown/P Gain", lastP);
            double i = SmartDashboard.getNumber("UpDown/I Gain", lastI);
            double d = SmartDashboard.getNumber("UpDown/D Gain", lastD);

            if (p != lastP || i != lastI || d != lastD) {
                SparkMaxConfig updatedConfig = new SparkMaxConfig();
                updatedConfig.closedLoop.p(p).i(i).d(d);
                intakeUpDownMotor.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                lastP = p;
                lastI = i;
                lastD = d;
            }
        }
    }

}