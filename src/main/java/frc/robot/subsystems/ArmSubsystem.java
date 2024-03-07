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

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {

    private static final String printLocation = "ArmSubsystem: ";
    private long startMovementTime;

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
    private ScheduledExecutorService taskExecutor;

    public ArmSubsystem() {
      // WHY ARE WE NOT SETTING FACTORY DEFAULTS FOR ARM MOTORS?
      // When your program starts up
      taskExecutor = Executors.newSingleThreadScheduledExecutor();

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
        leadScrewEncoder.setPosition(0);
        leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    
        leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 255);
        leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);
        leadScrewMotor.setIdleMode(IdleMode.kCoast);


        leadController.setP(LeadkP);
        leadController.setI(LeadkI);
        leadController.setD(LeadkD);
        leadController.setIZone(LeadkIz);
        leadController.setFF(LeadkFF);
        leadController.setOutputRange(LeadkMinOutput, LeadkMaxOutput);

        leadScrewMotor.burnFlash();



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

        /* // This is a simple delay task that stops the motor after a 5 second delay in case the PID is working overtimme
        // PREVENT LEAD SCREW MOTOR BURNOUT:
        Runnable checkMotorTask = () -> leadScrewStopMotor(orientation);
        //run this task after 5 seconds, nonblock for task3
        taskExecutor.schedule(checkMotorTask, 5, TimeUnit.SECONDS); */
      }
    }

    public void leadScrewStopMotor(Orientations orientation){
      System.out.println(printLocation+"*** leadScrewPosition told to Stop called by: "+orientation.label);   
      leadScrewMotor.set(0);
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