// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.OrientationConstants.Orientations;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {

    private static final String printLocation = "ArmSubsystem: ";

    // declare the arm motors
    private final static CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    // declare the leadScrew motor 550
    private final static CANSparkMax leadScrewMotor = new CANSparkMax(13, MotorType.kBrushless);
    
    // declare encoders for both
    public final static RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
    public final static RelativeEncoder leadScrewEncoder = leadScrewMotor.getEncoder();
    
    // declare PID Controllers for both 
    private final SparkPIDController armController = leftArmMotor.getPIDController();
    private final SparkPIDController leadController = leadScrewMotor.getPIDController();

    // Constants for PID control, adjust based on testing - NEEDS TUNING
    private boolean TUNING_MODE = false;

    double ArmkP = 0.1; // Proportional term
    double ArmkI = 0.0; // Integral term
    double ArmkD = 0.01; // Derivative term
    double ArmkIz = 0; // Integral zone
    double ArmkFF = 0.0; // Feed-forward
    double ArmkMaxOutput = .5;
    double ArmkMinOutput = -.3;

    //Lead screw 
    double LeadkP = 0.1; // Proportional term
    double LeadkI = 0.0; // Integral term
    double LeadkD = 0.01; // Derivative term
    double LeadkIz = 0; // Integral zone
    double LeadkFF = 0.0; // Feed-forward
    double LeadkMaxOutput = 1;
    double LeadkMinOutput = -1;
    //double LeadMaxAccel = 10;
    //double LeadMaxVelocity = 10;

    public float kLeadFarLimit = 255;
    public float kLeadHomeLimit = 0;

    public float kArmUpLimit = 52;
    public float kArmDownLimit = 0;

    public double armSetpoint = 0;
    public double leadSetpoint = 0;

    double kTuneLeadSetpoint = 150;

    // Gear reduction and encoder conversion factor - NEEDS UPDATE
    private static final double gearReduction = 197.142857143; // 197.142857143:1 Gear reduction (Not taking chain & Sprocket into account)
    private static final double encoderConversionFactor = 360.0 / gearReduction;

    public ArmSubsystem() {
      // WHY ARE WE NOT SETTING FACTORY DEFAULTS FOR ARM MOTORS?

      // rightArmMotor follows left and inverts output to the same axle (SUPER IMPORTANT)
      rightArmMotor.follow(leftArmMotor, true);

      // Set motors to brake mode
      leftArmMotor.setIdleMode(IdleMode.kCoast);
      rightArmMotor.setIdleMode(IdleMode.kCoast);

      leftArmMotor.setSmartCurrentLimit(40);
      rightArmMotor.setSmartCurrentLimit(40);

      // DO WE NEED A SOFT LIMIT ON THE RIGHT ARM?
      leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 52);
      leftArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);    

      leftArmMotor.burnFlash();
      rightArmMotor.burnFlash();

      // PID configuration
      armController.setP(ArmkP);
      armController.setI(ArmkI);
      armController.setD(ArmkD);
      armController.setIZone(ArmkIz);
      armController.setFF(ArmkFF);
      armController.setOutputRange(ArmkMinOutput, ArmkMaxOutput);

      // Arm Encoder setup
      leftArmEncoder.setPosition(0);


      //Lead Screw
      leadScrewMotor.restoreFactoryDefaults();
      leadScrewMotor.setSmartCurrentLimit(30);
      leadScrewMotor.setInverted(true);
      leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);  
      leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 255);
      leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);
      leadScrewMotor.setIdleMode(IdleMode.kCoast);
      
      // Lead Encoder setup
      leadScrewEncoder.setPosition(0);
      leadController.setFeedbackDevice(leadScrewEncoder);
      
      leadController.setP(LeadkP);
      leadController.setI(LeadkI);
      leadController.setD(LeadkD);
      //leadController.setIZone(LeadkIz);
      leadController.setFF(LeadkFF);
      leadController.setOutputRange(LeadkMinOutput, LeadkMaxOutput);
      //leadController.setSmartMotionMaxAccel(LeadMaxAccel, 0);
      //leadController.setSmartMotionMaxVelocity(LeadMaxVelocity, 0);

      
      leadScrewMotor.burnFlash();

      if (TUNING_MODE){
        SmartDashboard.putNumber("P Gain", LeadkP);
        SmartDashboard.putNumber("I Gain", LeadkI);
        SmartDashboard.putNumber("D Gain", LeadkD);
        SmartDashboard.putNumber("Feed Forward", LeadkFF);
        SmartDashboard.putNumber("Max Output", LeadkMaxOutput);
        SmartDashboard.putNumber("Min Output", LeadkMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
      }

      // display PID coefficients on SmartDashboard
    

    }

    public void moveToPosition(Orientations orientation) {
        setArmPosition(orientation.armPosition);
    }

    public void setArmPosition(double armPosition) {
      System.out.println("**ARM TRYING TO MOVE TO" + armPosition);
        // move to the position dynamically
        // need method to VERIFY armPosition is SAFE (within bounds) before using!!!!
        armSetpoint = armPosition;
    }

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
    
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);
    if (TUNING_MODE){
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        if((p!=LeadkP)) {leadController.setP(p); LeadkP = p; }
        if((i!=LeadkI)) {leadController.setP(i); LeadkI = i; }
        if((d!=LeadkD)) {leadController.setD(d); LeadkD = d; }
        if((ff!=LeadkFF)) {leadController.setFF(ff); LeadkFF = ff; }
        if((max!=LeadkMaxOutput)||(min!=LeadkMinOutput)) {
          leadController.setOutputRange(min, max); 
          LeadkMinOutput = min; 
          LeadkMaxOutput = max;
        }
        
    }
    armController.setReference(armSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID
    leadController.setReference(leadSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID

    SmartDashboard.putNumber("Lead Screw SetPoint:", rotations);
    SmartDashboard.putNumber("Lead Screw Encoder:", ArmSubsystem.leadScrewEncoder.getPosition());
  }
}