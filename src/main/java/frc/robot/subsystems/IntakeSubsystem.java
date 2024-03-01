// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {    
    
    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeMotor.burnFlash();
    
}

     public void runIntake(){

      intakeMotor.set(1);
      

    }
    public void stopIntake(){
      
      intakeMotor.set(0);
      

    }

    public void reverseIntake() {

      intakeMotor.set(-1);
    }
  

  @Override
  public void periodic() {

   
    // This method will be called once per scheduler run
  }
}
