// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/* 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import java.util.concurrent.ScheduledExecutorService;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.RobotContainer; */

public class LeadScrewSubsystem extends SubsystemBase {

  /* //private static final RobotContainer robotContainer = new RobotContainer();
  private final CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();
  private final SparkPIDController leadController = leadScrewMotor.getPIDController();
  private static final String printLocation = "LeadScrewSubsystem: ";

  //Lead screw 
    double LeadkP = 0.125; // Proportional term
    double LeadkI = 0.0; // Integral term
    double LeadkD = 0.00; // Derivative term
    double LeadkIz = 0; // Integral zone
    double LeadkFF = 0.0; // Feed-forward
    double LeadkMaxOutput = 1;
    double LeadkMinOutput = -1;
    //double LeadMaxAccel = 10;
    //double LeadMaxVelocity = 10;

    public float kLeadFarLimit = 581;
    public float kLeadHomeLimit = -1;

  double leadSetpoint = 0;
  double kTuneLeadSetpoint = 150;
  ScheduledExecutorService taskExecutor; */

  public LeadScrewSubsystem() {

    
    
    /* taskExecutor = Executors.newSingleThreadScheduledExecutor();

    // declare the leadScrew motor 550
    private final static CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);
    public final static RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();
    
    private final SparkPIDController leadController = leadScrewMotor.getPIDController();


    public double leadSetpoint = 0;

    double kTuneLeadSetpoint = 150;

    //Lead Screw
        leadScrewMotor.restoreFactoryDefaults();

        leadScrewMotor.setSmartCurrentLimit(30, 30);
        leadScrewMotor.setInverted(true);
        leadScrewEncoder.setPosition(0);
        if (CONTROL_MANUALLY){
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        } else {
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
          leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 581);
          leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);
        
        }

    
        leadScrewMotor.setIdleMode(IdleMode.kCoast);


        leadController.setP(LeadkP);
        leadController.setI(LeadkI);
        leadController.setD(LeadkD);
        leadController.setIZone(LeadkIz);
        leadController.setFF(LeadkFF);
        leadController.setOutputRange(LeadkMinOutput, LeadkMaxOutput);

        /* // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", LeadkP);
        SmartDashboard.putNumber("I Gain", LeadkI);
        SmartDashboard.putNumber("D Gain", LeadkD);
        SmartDashboard.putNumber("I Zone", LeadkIz);
        SmartDashboard.putNumber("Feed Forward", LeadkFF);
        SmartDashboard.putNumber("Max Output", LeadkMaxOutput);
        SmartDashboard.putNumber("Min Output", LeadkMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0); 

        leadScrewMotor.burnFlash();

    leadScrewMotor.setSmartCurrentLimit(30, 30);
    leadScrewMotor.setInverted(true);
    leadScrewEncoder.setPosition(0);

    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kLeadFarLimit);
    leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kLeadHomeLimit);

    leadScrewMotor.setIdleMode(IdleMode.kCoast);

    leadController.setP(LeadkP);
    leadController.setI(LeadkI);
    leadController.setD(LeadkD);
    leadController.setIZone(LeadkIz);
    leadController.setFF(LeadkFF);
    leadController.setOutputRange(LeadkMinOutput, LeadkMaxOutput);

    leadScrewMotor.burnFlash(); */

    /* // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", LeadkP);
        SmartDashboard.putNumber("I Gain", LeadkI);
        SmartDashboard.putNumber("D Gain", LeadkD);
        SmartDashboard.putNumber("I Zone", LeadkIz);
        SmartDashboard.putNumber("Feed Forward", LeadkFF);
        SmartDashboard.putNumber("Max Output", LeadkMaxOutput);
        SmartDashboard.putNumber("Min Output", LeadkMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0); */

      
  } 

  /* 
  public void leadMoveToPosition(Orientations orientation) {
      double position = orientation.leadScrewPosition;
      //System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
      if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
        leadSetpoint = position;

         // This is a simple delay task that stops the motor after a 5 second delay in case the PID is working overtimme
        // PREVENT LEAD SCREW MOTOR BURNOUT:
        if ((orientation.label == Orientations.HOME.label)||(orientation.label == Orientations.PRECLIMB.label)){
          Runnable checkMotorTask = () -> leadScrewStopMotor(orientation);
          //run this task after 5 seconds, nonblock for task3
          taskExecutor.schedule(checkMotorTask, 5, TimeUnit.SECONDS); 
        }

      }
    }

    public void leadScrewStopMotor(Orientations orientation){
      //System.out.println(printLocation+"*** leadScrewPosition told to Stop called by: "+orientation.label);   

    }
  
    public void leadScrewSetPosition(double position){
      //System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
      if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
        leadSetpoint = position;
      }
    }

    public void leadScrewForward(){
      leadScrewMotor.set(.2);
    }

    public void leadScrewStop(){

      leadScrewMotor.set(0);
    }
  
  
    public void leadScrewBackward() {
        leadScrewMotor.set(-.2);
    }
  public void leadMoveToPosition(Orientations orientation) {
    double position = orientation.leadScrewPosition;
    //System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
    if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
      leadSetpoint = position;
      if ((orientation.label == Orientations.HOME.label)||(orientation.label == Orientations.PRECLIMB.label)){
        Runnable checkMotorTask = () -> leadScrewStopMotor(orientation);
        //run this task after 5 seconds
        taskExecutor.schedule(checkMotorTask, 5, TimeUnit.SECONDS); 
      }

    }
  }

  // called when orientation goes to HOME or PRECLIMB and turns off the motor
  public void leadScrewStopMotor(Orientations orientation){
    System.out.println(printLocation+"*** leadScrewPosition told to Stop called by: "+orientation.label);   
  }

  public void leadScrewSetPosition(double position){
    System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
    if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
      leadSetpoint = position;
    }
  }

  public void leadScrewForward(){
    leadScrewMotor.set(.2);
  }

  public void leadScrewStop(){
    leadScrewMotor.set(0);
  }

  public void leadScrewBackward() {
      leadScrewMotor.set(-.2);
  } */

  @Override
  public void periodic() {
    /* SmartDashboard.putNumber("Lead Motor Temp", leadScrewMotor.getMotorTemperature());
    SmartDashboard.putNumber("Output voltage", leadScrewMotor.getOutputCurrent());
    SmartDashboard.putNumber("AppliedOutput", leadScrewMotor.getAppliedOutput());
    SmartDashboard.putNumber("Lead Screw Encoder", leadScrewEncoder.getPosition()); */

    /* // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != LeadkP)) { leadController.setP(p); LeadkP = p; }
    if((i != LeadkI)) { leadController.setI(i); LeadkI = i; }
    if((d != LeadkD)) { leadController.setD(d); LeadkD = d; }
    if((iz != LeadkIz)) { leadController.setIZone(iz); LeadkIz = iz; }
    if((ff != LeadkFF)) { leadController.setFF(ff); LeadkFF = ff; }
    if((max != LeadkMaxOutput) || (min != LeadkMinOutput)) { 
      leadController.setOutputRange(min, max); 
      LeadkMinOutput = min; LeadkMaxOutput = max; 
    }   
    /* SmartDashboard.putNumber("Lead Motor Temp", leadScrewMotor.getMotorTemperature());
    SmartDashboard.putNumber("Output voltage", leadScrewMotor.getOutputCurrent());
    SmartDashboard.putNumber("AppliedOutput", leadScrewMotor.getAppliedOutput());
    SmartDashboard.putNumber("Lead Screw Encoder", leadScrewEncoder.getPosition());

    leadController.setReference(leadSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID */
    /* if (!CONTROL_MANUALLY){
      leadController.setReference(leadSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID

    }
    else {
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
          leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        }
    
  } */
  }
}
