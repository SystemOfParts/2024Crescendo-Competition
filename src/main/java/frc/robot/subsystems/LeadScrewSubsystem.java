// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.RobotContainer;

public class LeadScrewSubsystem extends SubsystemBase {

  //private static final RobotContainer robotContainer = new RobotContainer();

  private final CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();
  private final SparkPIDController pidController = leadScrewMotor.getPIDController();

  public double kP = 1; // Proportional term
  public double kI = 0.0; // Integral term
  public double kD = 0.0; // Derivative term
  public double kIz = 0; // Integral zone
  public double kFF = 0.0; // Feed-forward
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  public float kFarLimit = 240;
  public float kHomeLimit = 1;

  private static final String printLocation = "LeadScrewSubsystem: ";
  public double setpoint = 0;

  public LeadScrewSubsystem() {
    
    leadScrewMotor.setSmartCurrentLimit(30);
    leadScrewEncoder.setPosition(0);

    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kFarLimit);
    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kHomeLimit);

    // set PID coefficients
    /* pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput); */

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    // Encoder setup
    leadScrewEncoder.setPosition(0);
      
  } 

  public void moveToPosition(Orientations orientation) {
    double position = orientation.leadScrewPosition;
    System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
    if ((position >= kHomeLimit) && (position <=kFarLimit)){
      setpoint = position;
    }
  }

  public void leadScrewSetPosition(double position){
    System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
    if ((position >= kHomeLimit) && (position <=kFarLimit)){
      setpoint = position;
    }
  }

  public void leadScrewForward(){
    leadScrewMotor.set(.8);
  }

  public void leadScrewStop(){
    leadScrewMotor.set(0);  
  }

  public void leadScrewBackward() {
      leadScrewMotor.set(-.8);
  }


  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
   /*  if (robotContainer.getYButton()) {

      leadScrewMotor.set(1);
    }
    else if (robotContainer.getAButton()){

      leadScrewMotor.set(-1);
    }
    else {

      leadScrewMotor.set(0);
    } */
   
    // This method will be called once per scheduler run
    pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }
}
