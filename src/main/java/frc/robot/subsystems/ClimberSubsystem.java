// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {

  private static final int climber1ID = 16;
  private static final int climber2ID = 17;

  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  climberMotor1 = new CANSparkMax(climber1ID, MotorType.kBrushless);
  climberMotor2 = new CANSparkMax(climber2ID, MotorType.kBrushless);

  climberMotor1.restoreFactoryDefaults();
  climberMotor2.restoreFactoryDefaults();
  }

  public void climber1Up() {
    System.out.println("climber1Up run");
    climberMotor1.set(-.2);
  }
  public void climber1Stop() {
    System.out.println("climber1Stop run");
    climberMotor1.set(0);
  }
  public void climber1Down() {
    System.out.println("climber1Down run");
    climberMotor1.set(.2);
  }
  public void climber2Up() {
    System.out.println("climber2Up run");
    climberMotor2.set(-.2);
  }
  public void climber2Stop() {
    System.out.println("climber2Stop run");
    climberMotor2.set(0);
  }
  public void climber2Down() {
    System.out.println("climber2Down run");
    climberMotor2.set(.2);
  }
  public void climberBrakeMode() {
    System.out.println("climberBrakeMode run");
    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
