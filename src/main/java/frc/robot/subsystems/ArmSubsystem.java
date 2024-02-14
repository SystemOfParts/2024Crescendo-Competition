// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftArmMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(12, MotorType.kBrushless);
    private final RelativeEncoder encoder = leftArmMotor.getEncoder();
    private final SparkPIDController pidController = leftArmMotor.getPIDController();

    // Constants for PID control, adjust based on testing - NEEDS TUNING
    private static final double kP = 0.1; // Proportional term
    private static final double kI = 0.0; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 0; // Integral zone
    private static final double kFF = 0.0; // Feed-forward
    private static final double kMaxOutput = 1;
    private static final double kMinOutput = -1;
    private static final double maxRPM = 5700; // Max RPM for NEO 500

    // Gear reduction and encoder conversion factor - NEEDS UPDATE
    private static final double gearReduction = 197.142857143; // 60:1 Gear reduction (Not taking chain & Sprocket into account)
    private static final double encoderConversionFactor = 360.0 / gearReduction; // Needs changing to convert encoder ticks into degrees
    private static final double degreesPerCount = 360 / (gearReduction * 42);

    public ArmSubsystem() {
        // rightArmMotor follows left and inverts output to the same axle (SUPER IMPORTANT)
        rightArmMotor.follow(leftArmMotor, true);

        // Set motors to brake mode
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);

        // PID configuration - NEEDS UPDATE
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Encoder setup
        encoder.setPositionConversionFactor(encoderConversionFactor); // Refers to degrees, i.e, degrees
        encoder.setVelocityConversionFactor(encoderConversionFactor / 60); // Sixty refers to seconds, i.e, degrees per second.
    }

    public void armDown() {
        moveToAngle(0);
    }

    public void armTo45Degrees() {
        moveToAngle(45);
    }

    public void armTo90Degrees() {
        moveToAngle(90);
    }

    public void armStop() {
        leftArmMotor.set(0);
    }

    private void moveToAngle(double angle) {
        double targetPosition = angle / degreesPerCount;
        pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}