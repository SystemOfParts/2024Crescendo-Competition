// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climberMotor1 = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax climberMotor2 = new CANSparkMax(17, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder1 = climberMotor1.getEncoder();
  private final RelativeEncoder climberEncoder2 = climberMotor2.getEncoder();

  private static final double GEAR_REDUCTION = 64.0;
  private static final double ENCODER_RESOLUTION = 42.0; // NEO 550 encoder counts per revolution
  private static final double AXLE_ROTATION_DISTANCE = 1.0; // Placeholder value for distance traveled per axle rotation (Centimeters)
  private static final double AXLE_REVOLUTIONS_TO_MAX = 1.0; // Placeholder value for total axle revolutions to maximum value
  private static final double MAX_POSITION = ENCODER_RESOLUTION * GEAR_REDUCTION * AXLE_ROTATION_DISTANCE * AXLE_REVOLUTIONS_TO_MAX; // Calculate max position
  private static final double MIN_POSITION = 0.0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberMotor1.restoreFactoryDefaults();
    climberMotor2.restoreFactoryDefaults();

    climberMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climberMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    climberMotor1.burnFlash();
    climberMotor2.burnFlash();

    climberEncoder1.setPosition(0);
    climberEncoder2.setPosition(0);

    }

    public void climber1Up(double speed) {
      if (climberEncoder1.getPosition() < MAX_POSITION) {
          climberMotor1.set(speed);
      } else {
          climberMotor1.set(0);
      }
  }

  public void climber1Down(double speed) {
      if (climberEncoder1.getPosition() > MIN_POSITION) {
          climberMotor1.set(-speed);
      } else {
          climberMotor1.set(0);
      }
  }

  public void climber2Up(double speed) {
      if (climberEncoder2.getPosition() < MAX_POSITION) {
          climberMotor2.set(speed);
      } else {
          climberMotor2.set(0);
      }
  }

  public void climber2Down(double speed) {
      if (climberEncoder2.getPosition() > MIN_POSITION) {
          climberMotor2.set(-speed);
      } else {
          climberMotor2.set(0);
      }
  }

  public void stopClimbers() {
    climberMotor1.set(0);
    climberMotor2.set(0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoderleft", climberEncoder1.getPosition());
    SmartDashboard.putNumber("Encoderright", climberEncoder2.getPosition());
    
    // This method will be called once per scheduler run
  }
}
