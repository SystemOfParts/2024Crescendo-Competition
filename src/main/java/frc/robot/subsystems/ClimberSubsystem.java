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

  private static final int climber1ID = 16;
  private static final int climber2ID = 17;

  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;

    private RelativeEncoder m_encoder1;
    private RelativeEncoder m_encoder2;
   public double arm1zero;
    public double arm2zero;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  
  climberMotor1 = new CANSparkMax(climber1ID, MotorType.kBrushless);
  climberMotor2 = new CANSparkMax(climber2ID, MotorType.kBrushless);

    m_encoder1 = climberMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_encoder2 = climberMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

  climberMotor1.restoreFactoryDefaults();
  climberMotor2.restoreFactoryDefaults();
  climberMotor1.setIdleMode(IdleMode.kBrake);
  climberMotor2.setIdleMode(IdleMode.kBrake);

  climberMotor1.burnFlash();
  climberMotor2.burnFlash();
   
  arm1zero = m_encoder1.getPosition();
  arm2zero = m_encoder2.getPosition();

    }
  public double getarm1position() {
    return (m_encoder1.getPosition()-arm1zero);
  }
   public double getarm2position() {
    return (m_encoder2.getPosition()-arm2zero);
  }

  public void climber1Up() {
    
    climberMotor1.set(-.25);
  }
  public void climber1Stop() {
    climberMotor1.set(0);
  }
  public void climber1Down() {

    if (getarm1position() < -5){
    climberMotor1.set(.25);
    }

  }
  public void climber2Up() {
    climberMotor2.set(-.25);
    }
  
  public void climber2Stop() {
    climberMotor2.set(0);
  }
  public void climber2Down() {
      if (getarm2position() < -5){
    climberMotor2.set(.25);
       }
  }

  
  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoderleft", m_encoder1.getPosition());
        SmartDashboard.putNumber("Encoderright", m_encoder2.getPosition());

    
    // This method will be called once per scheduler run
  }
}
