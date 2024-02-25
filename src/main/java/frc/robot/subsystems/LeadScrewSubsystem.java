// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Robot;
import frc.robot.RobotContainer;
public class LeadScrewSubsystem extends SubsystemBase {

  private static final RobotContainer robotContainer = new RobotContainer();

    private final CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public LeadScrewSubsystem() {

    leadScrewMotor.setSmartCurrentLimit(30);
    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 235);
    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);


  } //these methods dont work

     public void leadScrewForward(){

      leadScrewMotor.set(.3);
      

    }
    public void leadScrewStop(){
      
      leadScrewMotor.set(0);
      

    }

    public void leadScrewBackward() {

      leadScrewMotor.set(-.3);
    }


  @Override
  public void periodic() {

    if (robotContainer.getYButton()) {

    leadScrewMotor.set(.3);
    }
    else if (robotContainer.getAButton()){

      leadScrewMotor.set(-.3);
    }
    else {

      leadScrewMotor.set(0);
    }
   
    // This method will be called once per scheduler run
  }
}
