// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax topShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);


  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    bottomShooterMotor.follow(topShooterMotor, true);

  }

     public void runShooter(){

      topShooterMotor.set(.5);
      
    }
    public void stopShooter(){
      
      topShooterMotor.set(0);
      
    }
  

  @Override
  public void periodic() {

   
    // This method will be called once per scheduler run
  }
}

