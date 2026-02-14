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
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.*;



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

    /**
     * Constructs a new Intake subsystem.
     * Initializes the intake motor.
     */
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Constants.RobotConstants.kIntakeMotorID, MotorType.kBrushless);

        intakeUpDownMotor = new SparkMax(Constants.RobotConstants.kIntakeUpDownMotorID, MotorType.kBrushless);
        intakeUpDownEncoder = intakeUpDownMotor.getEncoder();
        intakeUpDownPID = intakeUpDownMotor.getClosedLoopController();

        intakeUpDownEncoder.setPosition(0.0);

    }

    /**
     * Starts the intake motor.
     */
    public void startIntake() {

        intakeMotor.set(RobotConstants.kIntakeSpeed);

    }

    /**
     * Starts the intake motor, but backwards.
     */
    public void reverseIntake() {

    intakeMotor.set(-RobotConstants.kIntakeSpeed);

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
     * Gets the RPM of the intake motor, converted using a conversion factor.
     * 
     * @return The converted RPM of the intake motor.
     */
    public double getRPM(){

        return getRawMotorRPM() * RobotConstants.kRPMConversionFactor;

    }

    /**
     * Moves the intake down.
     */
    public void moveIntakeDown() {
    intakeUpDownPID.setSetpoint(RobotConstants.kDownPosition, ControlType.kPosition);
}

    /**
     * Moves the intake up.
     */
    public void moveIntakeUp() {
    intakeUpDownPID.setSetpoint(RobotConstants.kUpPosition, ControlType.kPosition);
}

    /**
     * Periodically updates the SmartDashboard with the intake motor's RPM and current.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic() {
        if (UtilityConstants.debugMode){
            SmartDashboard.putNumber("Intake RPM", getRPM());
            SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        }
    }

}