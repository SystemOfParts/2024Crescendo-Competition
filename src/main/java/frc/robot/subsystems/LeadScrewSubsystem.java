// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OrientationConstants.Orientations;

public class LeadScrewSubsystem extends SubsystemBase {
  
    private final CANSparkMax leadScrewMotor = new CANSparkMax(12, MotorType.kBrushless);
    private final RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();
  
    private static final double MAX_POSITION = 247;
    private static final double MIN_POSITION = 5;


  /** Creates a new LeadScrewSubsystem. */
  public LeadScrewSubsystem() {

    leadScrewMotor.restoreFactoryDefaults();
    leadScrewEncoder.setPosition(0);
    leadScrewMotor.setInverted(true);
    leadScrewMotor.setSmartCurrentLimit(30);
    leadScrewMotor.burnFlash();

  }

  public void leadScrewForward(){ 
   
    if (leadScrewEncoder.getPosition() < MAX_POSITION) {

    leadScrewMotor.set(.1);  

    }
    else {
    leadScrewMotor.set(0);
    }
  }


  public void leadScrewBackward(){

    if (leadScrewEncoder.getPosition() > MIN_POSITION) {

    leadScrewMotor.set(-.1);

    }
    else {
    leadScrewMotor.set(0);
    }

  }  
  

  public void leadScrewStop(){
    leadScrewMotor.set(0);
  }

  public void moveToPosition(Orientations orientation) {
        setLeadScrewPosition(orientation.leadScrewPosition);
    }

    public void setLeadScrewPosition(double leadScrewPosition) {
      System.out.println("**LEADSCREW TRYING TO MOVE TO" + leadScrewPosition);
      // move to the position dynamically
      // need method to VERIFY extensionPosition is SAFE (within bounds) before using!!!!
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(leadScrewEncoder.getPosition());
  }
}
