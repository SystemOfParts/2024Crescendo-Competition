// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.OrientationConstants.Orientations;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    private final RelativeEncoder encoder = leftArmMotor.getEncoder();
    private final SparkPIDController pidController = leftArmMotor.getPIDController();

    // Constants for PID control, adjust based on testing - NEEDS TUNING
    private static final double kP = 0.1; // Proportional term
    private static final double kI = 0.0; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 0; // Integral zone
    private static final double kFF = 0.0; // Feed-forward
    private static final double kMaxOutput = .3;
    private static final double kMinOutput = -.3;
    public double setpoint = 0;

    // Gear reduction and encoder conversion factor - NEEDS UPDATE
    private static final double gearReduction = 197.142857143; // 197.142857143:1 Gear reduction (Not taking chain & Sprocket into account)
    private static final double encoderConversionFactor = 360.0 / gearReduction;

    private static final String printLocation = "ArmSubsystem: ";

    public ArmSubsystem() {
        // rightArmMotor follows left and inverts output to the same axle (SUPER IMPORTANT)
        rightArmMotor.follow(leftArmMotor, true);

        // Set motors to brake mode
        leftArmMotor.setIdleMode(IdleMode.kCoast);
        rightArmMotor.setIdleMode(IdleMode.kCoast);

        leftArmMotor.setSmartCurrentLimit(40);
        rightArmMotor.setSmartCurrentLimit(40);

        leftArmMotor.burnFlash();
        rightArmMotor.burnFlash();

        // PID configuration
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Encoder setup
        encoder.setPosition(0);

    }

    public void armDown() {
        setpoint = 3.5/encoderConversionFactor;
    }

    public void armTo45Degrees() {
        setpoint = 15/encoderConversionFactor;

    }

    public void armTo85Degrees() {
        setpoint = 45/encoderConversionFactor;

    }

    public void armToAmp() {
        setpoint = 90/encoderConversionFactor;
    }

    public void moveToPosition(Orientations orientation) {
      double position = orientation.armPosition;
      System.out.println(printLocation+"*** Position called to: "+position);   
      setpoint = position/encoderConversionFactor;
    }

    public void setArmPosition(double armPosition) {
      System.out.println("**ARM TRYING TO MOVE TO" + armPosition);
        // move to the position dynamically
        // need method to VERIFY armPosition is SAFE (within bounds) before using!!!!
    }

    public void armStop() {
        leftArmMotor.set(0);
    }

    public void armMoveUp() {
      leftArmMotor.set(.25);
    }
    public void armMoveDown() {
      leftArmMotor.set(-.25);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//System.out.print(encoder.getPosition());

    //System.out.println("encoder position: ");

    pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID

  }
}