// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.Relay;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climberMotor1 = new CANSparkMax(Constants.CanIdConstants.kClimber1Id, MotorType.kBrushless);
  private final CANSparkMax climberMotor2 = new CANSparkMax(Constants.CanIdConstants.kClimber2Id, MotorType.kBrushless);

  
  private final Relay leftBrakeRelay = new Relay(0);
  private final Relay rightBrakeRelay = new Relay(1);

  public boolean ClimbingModeOn = false;

  public boolean manualModeOn = false;

  private final RelativeEncoder climberEncoder1;
  private final RelativeEncoder climberEncoder2;

  private static final double MAX_POSITION = -375;//-400.0
  private static final double MAX_POSITION_LEFT = -570.0;//-400.0
  private static final double MIN_POSITION = Constants.ClimberConstants.kMIN_POSITION;//-10

  /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {

        climberEncoder1 = climberMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        climberEncoder2 = climberMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42); // this takes into account encoder resolution

        climberMotor1.restoreFactoryDefaults();
        climberMotor2.restoreFactoryDefaults();

        climberMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        climberMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        climberMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        climberMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        climberMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, -5); //-375
        climberMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, -5); //-570
        climberMotor1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -570); //-5
        climberMotor2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -375); //-5

        climberMotor1.setSmartCurrentLimit(30);
        climberMotor2.setSmartCurrentLimit(30);

        climberMotor2.setInverted(true);

        climberMotor1.burnFlash();
        climberMotor2.burnFlash();

        climberEncoder1.setPosition(0);
        climberEncoder2.setPosition(0);

        leftBrakeRelay.setDirection(Relay.Direction.kBoth);        
        rightBrakeRelay.setDirection(Relay.Direction.kBoth);

    
    }
    
    public void climber1Up(double speed) {

      if (ClimbingModeOn){
        
        if (climberEncoder1.getPosition() > MAX_POSITION_LEFT) {
            climberMotor1.set(-speed);
        } else {
            climberMotor1.set(0);
        }
      }
    }
      public void climber2Up(double speed) {
        if (ClimbingModeOn){
          if (climberEncoder2.getPosition() > MAX_POSITION) {
              climberMotor2.set(-speed);
          } else {
            climberMotor2.set(0);
          }
      }
    }
    

  public void climber1Down(double speed) {
      if (ClimbingModeOn){
          if (manualModeOn) {

                climberMotor1.set(speed);

          }
        else if (climberEncoder1.getPosition() < MIN_POSITION) { //true)
            climberMotor1.set(speed);
        } 
        else {
            climberMotor1.set(0);
        }
      }
    }
  public void climber2Down(double speed) {
      if (ClimbingModeOn){

        if (manualModeOn){
          climberMotor2.set(speed);
        }
        else if (climberEncoder2.getPosition() < MIN_POSITION) {
            climberMotor2.set(speed);

        } else {
            climberMotor2.set(0);
        }
      }
    }
    //dangerous controls for testing
    
  public void climber1DownManually(double speed){
          if (ClimbingModeOn){

    climberMotor1.set(speed);
          }
  }
  public void climber1UpManually(double speed){
          if (ClimbingModeOn){

    climberMotor1.set(-speed);
          }
  }
   public void climber2DownManually(double speed){
          if (ClimbingModeOn){

    climberMotor2.set(speed);
          }
  }
  public void climber2UpManually(double speed){
          if (ClimbingModeOn){

    climberMotor2.set(-speed);
          }
  }

   // Returns the climber position
    public double getClimber1Position() {
        return climberEncoder1.getPosition();
    }

    public double getClimber2Position() {
        return climberEncoder2.getPosition();
    }

  public void stopClimber1() {
          if (ClimbingModeOn){

    climberMotor1.set(0);
          }
  }
  public void stopClimber2() {
          if (ClimbingModeOn){

    climberMotor2.set(0);
          }
  }

  public void leftBrakeOn() {
    if (ClimbingModeOn){
      leftBrakeRelay.set(Relay.Value.kForward);
    }
  }

  public void rightBrakeOn() {
    if (ClimbingModeOn){
      leftBrakeRelay.set(Relay.Value.kReverse);
    }
  }

  public void ClimberModeTurnOn() {
    //System.out.println("Turned on climber mode");
  
    ClimbingModeOn = true;
  }
  
  
  public void ClimberModeTurnOff() {
    //System.out.println("Turned off climber mode");
  
    ClimbingModeOn = false;
  }
  
  public boolean getClimberMode() {
  
    //System.out.println("Accessed Climbing Mode");
    
    return ClimbingModeOn;
  }
  public void manualModeTurnOn() {
    //System.out.println("Turned on climber mode");
  
    manualModeOn = true;
     climberMotor1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
          climberMotor2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  }
  
  
  public void manualModeTurnOff() {
    //System.out.println("Turned off climber mode");
  
    manualModeOn = false;
   
   
  }
  
  public boolean getManualMode() {
  
    //System.out.println("Accessed Climbing Mode");
    
    return manualModeOn;
  }
  
  
  @Override
  public void periodic() {
    //System.out.println(climberEncoder2.getPosition());
    //System.out.println(climberEncoder1.getPosition());

    //System.out.println("Left:" + leftBrakeRelay.get());
    //System.out.println("Right:" + rightBrakeRelay.get());

  }
}
