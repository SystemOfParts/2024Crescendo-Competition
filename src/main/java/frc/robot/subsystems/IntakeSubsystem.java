// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.XboxRumbleCommand;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {

private final CANSparkMax intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
public static DigitalInput noteSensorRight;
public static DigitalInput noteSensorLeft;
public boolean isShooting = false;
public boolean isEjecting = false;
public boolean isIntaking = false;
public boolean isBlack = false;
//public boolean hasNote;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {    

  new Trigger(() -> isNoteInIntake())
  
   .onTrue(new XboxRumbleCommand(0.25, .25, RumbleType.kBothRumble));

    
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.burnFlash();
    
    try {
      noteSensorRight = new DigitalInput(1);
      noteSensorLeft = new DigitalInput(0);
      //System.out.println("*** NOTE SENSOR INITIALIZED ***");
    } catch (Exception e) {
      //System.out.println("*** NOTE SENSOR FAILED TO LOAD ***");
    }
  }

    // run the intake at full speed
    public void runIntake(Boolean bool){
      //System.out.println("RUNNING INTAKE");
      isShooting = bool;
      if (isShooting){
        RobotContainer.LEDs.setPattern(BlinkinPattern.CONFETTI);
      }
      isIntaking = true;  
      isEjecting = false;
      intakeMotor.set(1);
    }

    // stop the intake by setting the speed to 0
    public void stopIntake(Boolean delayStop){
      //System.out.println("************************************************ stopIntake Called  ***");
      isEjecting = false;
      isIntaking = false;
      isShooting = false;
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
    public boolean detectOnLeft() {
      return !noteSensorLeft.get();
    }

    public boolean detectOnRight() {
      return !noteSensorRight.get();
    }


    // tell us if a note has been detected
    public boolean isNoteInIntake() {
      if (!noteSensorRight.get() || !noteSensorLeft.get()){
        //System.out.println("isNoteInIntake was true: LEFT: "+noteSensorLeft.get()+" ----   RIGHT: "+noteSensorRight.get());
        RobotContainer.LEDs.setPattern(BlinkinPattern.GREEN);
        isBlack = false;

      } 
      else {
        if (!isBlack){
          RobotContainer.LEDs.setPattern(BlinkinPattern.DARK_RED);
          isBlack = true;
        }
        //System.out.println("isNoteInIntake was FALSE: LEFT: "+noteSensorLeft.get()+" ----   RIGHT: "+noteSensorRight.get());
      }
      return  (!noteSensorRight.get() || !noteSensorLeft.get());
    }

  @Override
  public void periodic() {    
    //System.out.println("************************************************ !isNoteInIntake?  ***: "+!isNoteInIntake());
    //System.out.println("************************************************ !isShooting?  ***: "+!isShooting);
    if (isNoteInIntake() && isIntaking){
      if ((!isShooting)&&(!isEjecting)){
        //System.out.println("************************************************ stopping intake  ***");
        isIntaking = false;
        //new IntakeStopCommand(RobotContainer.intakeSubsystem, true);
        stopIntake(false);
      }
    }
  }
}