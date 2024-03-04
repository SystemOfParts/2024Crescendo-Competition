// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
public static DigitalInput noteSensor;
public boolean isShooting = false;
//public boolean hasNote;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {    
    
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
    
    try {
      noteSensor = new DigitalInput(0);
      System.out.println("*** NOTE SENSOR INITIALIZED ***");
    } catch (Exception e) {
      System.out.println("*** NOTE SENSOR FAILED TO LOAD ***");
    }
  }

    // run the intake at full speed
    public void runIntake(Boolean bool){
      isShooting = bool;
      intakeMotor.set(1);      
      System.out.println("Shooting");

    }

    // stop the intake by setting the speed to 0
    public void stopIntake(){
      intakeMotor.set(0);
    }

    // eject from the intake by reversing its direction
    public void reverseIntake() {
      intakeMotor.set(-1);
    }

    // tell us if a note has been detected
    public boolean isNoteInIntake() {
      return  noteSensor.get();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("************************************************ !isNoteInIntake?  ***: "+!isNoteInIntake());
    //System.out.println("************************************************ !isShooting?  ***: "+!isShooting);
    if (!isNoteInIntake()){
      if (!isShooting){
        //System.out.println("************************************************ stopping intake  ***");
        stopIntake();
      }
    }
  }
}