// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
public static DigitalInput noteSensorRight;
public static DigitalInput noteSensorLeft;
public boolean isShooting = false;
public boolean isEjecting = false;
public boolean isIntaking = false;
//public boolean hasNote;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {    
    
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
    
    try {
      noteSensorRight = new DigitalInput(0);
      noteSensorLeft = new DigitalInput(1);
      //System.out.println("*** NOTE SENSOR INITIALIZED ***");
    } catch (Exception e) {
      //System.out.println("*** NOTE SENSOR FAILED TO LOAD ***");
    }
  }

    // run the intake at full speed
    public void runIntake(Boolean bool){
      isShooting = bool;
      isIntaking = true;  
      isEjecting = false;
      intakeMotor.set(1);
      //System.out.println("Shooting");
    }

    // stop the intake by setting the speed to 0
    public void stopIntake(Boolean delayStop){
      isEjecting = false;
      isIntaking = false;
      isShooting = false;
      if (delayStop){
        new WaitCommand(.25);
      }
      intakeMotor.set(0);
    }

    // eject from the intake by reversing its direction
    public void reverseIntake() {
      isEjecting = true;
      isIntaking = false;
      isShooting = false;
      intakeMotor.set(-1);
    }

    // tell us if a note has been detected
    public boolean isNoteInIntake() {
      return  (!noteSensorRight.get() || !noteSensorLeft.get());
    }

  @Override
  public void periodic() {    
    SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getMotorTemperature());
    // This method will be called once per scheduler run
    //System.out.println("************************************************ !isNoteInIntake?  ***: "+!isNoteInIntake());
    //System.out.println("************************************************ !isShooting?  ***: "+!isShooting);
    if (isNoteInIntake() && isIntaking){
      if ((!isShooting)&&(!isEjecting)){
        //System.out.println("************************************************ stopping intake  ***");
        stopIntake(true);
      }
    }
  }
}