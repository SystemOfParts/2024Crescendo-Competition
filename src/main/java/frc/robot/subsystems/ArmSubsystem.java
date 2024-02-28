// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.OrientationConstants.Orientations;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {

    private static final String printLocation = "LeadScrewSubsystem: ";

    private final CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);


    private final CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);
    
    private final RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();


    private final RelativeEncoder encoder = leftArmMotor.getEncoder();

    private final SparkPIDController armController = leftArmMotor.getPIDController();

    private final SparkPIDController leadController = leadScrewMotor.getPIDController();

    // Constants for PID control, adjust based on testing - NEEDS TUNING

    private static final double ArmkP = 0.1; // Proportional term
    private static final double ArmkI = 0.0; // Integral term
    private static final double ArmkD = 0.0; // Derivative term
    private static final double ArmkIz = 0; // Integral zone
    private static final double ArmkFF = 0.0; // Feed-forward
    private static final double ArmkMaxOutput = .3;
    private static final double ArmkMinOutput = -.3;



    //Lead screw 
    private static final double LeadkP = 0.1; // Proportional term
    private static final double LeadkI = 0.0; // Integral term
    private static final double LeadkD = 0.0; // Derivative term
    private static final double LeadkIz = 0; // Integral zone
    private static final double LeadkFF = 0.0; // Feed-forward
    private static final double LeadkMaxOutput = 1;
    private static final double LeadkMinOutput = -1;

    public float kLeadFarLimit = 245;
    public float kLeadHomeLimit = 1;
    public double armSetpoint = 0;
    public double leadSetpoint = 0;



    // Gear reduction and encoder conversion factor - NEEDS UPDATE
    private static final double gearReduction = 197.142857143; // 197.142857143:1 Gear reduction (Not taking chain & Sprocket into account)
    private static final double encoderConversionFactor = 360.0 / gearReduction;

    public ArmSubsystem() {
        // rightArmMotor follows left and inverts output to the same axle (SUPER IMPORTANT)
        rightArmMotor.follow(leftArmMotor, true);

        // Set motors to brake mode
        leftArmMotor.setIdleMode(IdleMode.kCoast);
        rightArmMotor.setIdleMode(IdleMode.kCoast);

        leftArmMotor.setSmartCurrentLimit(40);
        rightArmMotor.setSmartCurrentLimit(40);

        leftArmMotor.burnFlash();
        rightArmMotor.burnFlash();

        // PID configuration
        armController.setP(ArmkP);
        armController.setI(ArmkI);
        armController.setD(ArmkD);
        armController.setIZone(ArmkIz);
        armController.setFF(ArmkFF);
        armController.setOutputRange(ArmkMinOutput, ArmkMaxOutput);

        // Encoder setup
        encoder.setPosition(0);


        //Lead Screw
        

        leadScrewMotor.restoreFactoryDefaults();

        leadScrewMotor.setSmartCurrentLimit(30);
        leadScrewMotor.setInverted(true);
        leadScrewEncoder.setPosition(0);
        leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    
        leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 245);
        leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);

        leadController.setP(LeadkP);
        leadController.setI(LeadkI);
        leadController.setD(LeadkD);
        leadController.setIZone(LeadkIz);
        leadController.setFF(LeadkFF);
        leadController.setOutputRange(LeadkMinOutput, LeadkMaxOutput);

        leadScrewMotor.burnFlash();



    }

    /* public void armDown() {
        armSetpoint = 3.5/encoderConversionFactor;
    }

    public void armTo45Degrees() {
        armSetpoint = 15/encoderConversionFactor;

    }

    public void armTo85Degrees() {
        armSetpoint = 45/encoderConversionFactor;

    }

    public void armToAmp() {
        armSetpoint = 90/encoderConversionFactor;
    } */

    public void moveToPosition(Orientations orientation) {
        setArmPosition(orientation.armPosition);
    }

    public void setArmPosition(double armPosition) {
      System.out.println("**ARM TRYING TO MOVE TO" + armPosition);
        // move to the position dynamically
        // need method to VERIFY armPosition is SAFE (within bounds) before using!!!!
        armSetpoint = armPosition;
    }
/*
    public void armStop() {
        leftArmMotor.set(0);
    }

    public void armMoveUp() {
      leftArmMotor.set(.25);
    }
    public void armMoveDown() {
      leftArmMotor.set(-.25);
    }

     public void leadScrewForward(){
      leadScrewMotor.set(.3);
    }
    
    public void leadScrewStop(){ 
      leadScrewMotor.set(0);
    }

    public void leadScrewBackward() {
      leadScrewMotor.set(-.3);
    }

    public void leadScrewHome(){

      leadSetpoint = 1;
    }
    
    public void leadScrewHover(){

      leadSetpoint = 150;
    }

    
    public void leadScrewIntake(){

      leadSetpoint = 240;
    } */

     public void leadMoveToPosition(Orientations orientation) {
      double position = orientation.leadScrewPosition;
      System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
      if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
        leadSetpoint = position;
      }
    }
  
    public void leadScrewSetPosition(double position){
      System.out.println(printLocation+"*** leadScrewPosition called to: "+position);   
      if ((position >= kLeadHomeLimit) && (position <= kLeadFarLimit)){
        leadSetpoint = position;
      }
    }
  
    
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//System.out.print(encoder.getPosition());

    //System.out.println("encoder position: ");

    armController.setReference(armSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID
    leadController.setReference(leadSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID

  }
}