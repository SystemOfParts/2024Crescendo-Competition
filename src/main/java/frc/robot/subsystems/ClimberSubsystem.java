// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climberMotor1 = new CANSparkMax(16, MotorType.kBrushless);
  //private final CANSparkMax climberMotor2 = new CANSparkMax(17, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder1;
  //private final RelativeEncoder climberEncoder2;

  //private final SparkPIDController m_pidcontroller;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    double setpoint = 0;

  private static final double GEAR_REDUCTION = 64.0;
  //private static final double ENCODER_RESOLUTION = 42.0; // NEO 550 encoder counts per revolution // the encoder gives 1 per revolution
  //private static final double AXLE_ROTATION_DISTANCE = 1.27; // Placeholder value for distance traveled per axle rotation (Centimeters)
  //private static final double AXLE_REVOLUTIONS_TO_MAX = 80.0; // Placeholder value for total axle revolutions to maximum value
  //private static final double MAX_POSITION = -1*(GEAR_REDUCTION * AXLE_ROTATION_DISTANCE * AXLE_REVOLUTIONS_TO_MAX); // Calculate max position
  private static final double MAX_POSITION = -400.0;
  private static final double MIN_POSITION = 0.0;

  /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {

        climberEncoder1 = climberMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        //climberEncoder2 = climberMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42); // this takes into account encoder resolution

        climberMotor1.restoreFactoryDefaults();
        //climberMotor2.restoreFactoryDefaults();

        climberMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //climberMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberMotor1.burnFlash();
        //climberMotor2.burnFlash();

        climberEncoder1.setPosition(0);
        //climberEncoder2.setPosition(0);
/*
        m_pidcontroller = climberMotor1.getPIDController();
        m_pidcontroller.setFeedbackDevice(climberEncoder1);

        kP = .1;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.25;
        kMinOutput = -0.25;

        //set PID
        m_pidcontroller.setP(kP);
        m_pidcontroller.setI(kI);
        m_pidcontroller.setD(kD);
        m_pidcontroller.setIZone(kIz);
        m_pidcontroller.setFF(kFF);
        m_pidcontroller.setOutputRange(kMinOutput, kMaxOutput);

*/
    }

    public void moveToMiddle(){
        System.out.println("MOVE TO MIDDLE");
        //setpoint = -200;
        //m_pidcontroller.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    
    }

    public void moveToBottom(){
        System.out.println("MOVE TO Bottom");
        //setpoint = -100;
        //m_pidcontroller.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    
    }
    
    public void climber1Up(double speed) {
        System.out.println("POSITION: "+(climberEncoder1.getPosition()));
        System.out.println("MAX_POSITION: "+(MAX_POSITION));
        System.out.println("POS-> "+(climberEncoder1.getPosition())+" > "+MAX_POSITION+" <-MAX");
        if (climberEncoder1.getPosition() > MAX_POSITION) {
            System.out.println("GOING UP");
            climberMotor1.set(-speed);
        } else {
            climberMotor1.set(0);
        }
    }
  

  public void climber1Down(double speed) {
        System.out.println("POSITION: "+(climberEncoder1.getPosition()));
        System.out.println("MIN_POSITION: "+(MIN_POSITION));
        System.out.println("POS-> "+(climberEncoder1.getPosition())+" < "+MIN_POSITION+" <-MIN");
        if (climberEncoder1.getPosition() < MIN_POSITION) {
            System.out.println("GOING DOWN");
            climberMotor1.set(speed);
        } else {
            climberMotor1.set(0);
        }
    }

  public void climber1DownManually(double speed){
    System.out.println("POSITION: "+(climberEncoder1.getPosition()));
    climberMotor1.set(speed);
  }

  public void climber1UpManually(double speed){
    System.out.println("POSITION: "+(climberEncoder1.getPosition()));
    climberMotor1.set(-speed);
  }

   // Returns the climber position
    public double getClimberPosition() {
        return climberEncoder1.getPosition();
    }

  public void climber2Up(double speed) {
      /*if (climberEncoder2.getPosition() < MAX_POSITION) {
          climberMotor2.set(-speed);
      } else {
          climberMotor2.set(0);
      }
      */
  }

  public void climber2Down(double speed) {
      /*if (climberEncoder2.getPosition() > MIN_POSITION) {
          climberMotor2.set(speed);
      } else {
          climberMotor2.set(0);
      }
      */
  }

  public void stopClimber1() {
    climberMotor1.set(0);
  }
  public void stopClimber2() {
    //climberMotor2.set(0);
  }

  public void tunePIDs() {
    /* 
    double p = SmartDashboard.getNumber("Climber P Gain", 0);
    double i = SmartDashboard.getNumber("Climber I Gain", 0);
    double d = SmartDashboard.getNumber("Climber D Gain", 0);
    double iz = SmartDashboard.getNumber("Climber I Zone", 0);
    double ff = SmartDashboard.getNumber("Climber Feed Forward", 0);
    double min = SmartDashboard.getNumber("Climber MinOutput", 0);
    double max = SmartDashboard.getNumber("Climber MaxOutput", 0);
    
    if ((p != kP)){
        m_pidcontroller.setP(p);
        kP = p;
    }
    if ((i != kI)){
        m_pidcontroller.setI(i);
        kI = i;
    }
    if ((d != kD)){
        m_pidcontroller.setD(d);
        kD = d;
    }
    if ((iz != kIz)){
        m_pidcontroller.setIZone(iz);
        kIz = iz;
    }
    if ((ff != kFF)){
        m_pidcontroller.setFF(ff);
        kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)){
        m_pidcontroller.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
    }
    */
}

  @Override
  public void periodic() {
/* 
    SmartDashboard.putNumber("Climber P Gain", kP);
    SmartDashboard.putNumber("Climber I Gain", kI);
    SmartDashboard.putNumber("Climber D", kD);
    SmartDashboard.putNumber("Climber I Zone", kIz);
    SmartDashboard.putNumber("Climber Feed Forward", kFF);
    SmartDashboard.putNumber("Climber MinOutput", kMinOutput);
    SmartDashboard.putNumber("Climber MaxOutput", kMaxOutput);
    SmartDashboard.putNumber("Climber Position", getClimberPosition());
    
    SmartDashboard.putNumber("SetPoint", setpoint);
    SmartDashboard.putNumber("Encoderleft", climberEncoder1.getPosition());
    //SmartDashboard.putNumber("Encoderright", climberEncoder2.getPosition());
    
    tunePIDs();
   */ 
  }
}
