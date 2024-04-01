// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.RobotContainer;
import frc.robot.Constants.OrientationConstants.Orientations;

import frc.robot.Constants.FreakyConstants;


public class ShooterSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = false;

  private final CANSparkMax bottomShooterMotor;
  private final CANSparkMax topShooterMotor;
  
  private final SparkPIDController bottomShooterPID;
  private final SparkPIDController topShooterPID;
  
  private RelativeEncoder bottomShooterEncoder;
  private RelativeEncoder topShooterEncoder;
  
  //private boolean runOnce = true;

// PID constant for tuning
  double kP = 0; // Proportional term - Waterbury: .00045
  double kI = 0; // Integral term - Waterbury: .001
  double kD = 0; // Derivative term - Waterbury: .008
  double kIz = 0; // Integral zone - Waterbury: 10
  double kFF = 0.00025; // Feed-forward - Waterbury: .00025
  double kMaxOutput = 1; // Change these later - Waterbury: 1
  double kMinOutput = 0; //- Waterbury: -1
  //private double stopSpeed = 0;
  double requestedSetpoint = 0; //- Waterbury: 0
  double humSpeed = 1200; //- Waterbury: 500
  double setpoint = 0; //- Waterbury: 0
  double setpointTolerance = 100; //- Waterbury: 100

  boolean HumModeOff = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    topShooterPID = topShooterMotor.getPIDController();
    topShooterEncoder = topShooterMotor.getEncoder();
    
    topShooterMotor.restoreFactoryDefaults();
    topShooterMotor.clearFaults();
    //topShooterPID.setFeedbackDevice(topShooterEncoder);
    topShooterMotor.setIdleMode(IdleMode.kBrake);
    topShooterMotor.setSmartCurrentLimit(40);
    //topShooterEncoder.setPositionConversionFactor(2*Math.PI);
    //topShooterEncoder.setVelocityConversionFactor(2*Math.PI/60);
    topShooterMotor.setCANTimeout(0);
    topShooterMotor.enableVoltageCompensation(12.0);
    topShooterMotor.setOpenLoopRampRate(.25);
    topShooterMotor.setClosedLoopRampRate(.25);
    topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
    topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
    
    topShooterPID.setPositionPIDWrappingEnabled(false);
    topShooterPID.setP(kP);
    topShooterPID.setI(kI);
    topShooterPID.setD(kD);
    topShooterPID.setFF(kFF);
    topShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    
    bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    bottomShooterPID = bottomShooterMotor.getPIDController();
    bottomShooterEncoder = bottomShooterMotor.getEncoder();

    bottomShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.clearFaults();
    //bottomShooterPID.setFeedbackDevice(bottomShooterEncoder);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setSmartCurrentLimit(40);
    bottomShooterEncoder.setPositionConversionFactor(2*Math.PI);
    bottomShooterEncoder.setVelocityConversionFactor(2*Math.PI/60);
    bottomShooterMotor.setCANTimeout(0);
    bottomShooterMotor.enableVoltageCompensation(12.0);
    bottomShooterMotor.setOpenLoopRampRate(.25);
    bottomShooterMotor.setClosedLoopRampRate(.25);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);
    bottomShooterMotor.follow(topShooterMotor);
     
    bottomShooterPID.setPositionPIDWrappingEnabled(false);
    bottomShooterPID.setP(kP);
    bottomShooterPID.setI(kI);
    bottomShooterPID.setD(kD);
    bottomShooterPID.setFF(kFF);
    bottomShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    
    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();
    
    if (TUNING_MODE){
      
      addPIDToDashboard();
    }
  }
  
  public void HumModeTurnOff() {
    //System.out.println("Turned on climber mode");
  
    HumModeOff = true;
  }
   public void HumModeTurnOn() {
    //System.out.println("Turned on climber mode");
  
    HumModeOff = false;
  }

  public void addPIDToDashboard() {
    SmartDashboard.putNumber("kShooterP", kP);
    SmartDashboard.putNumber("kShooterI", kI);
    SmartDashboard.putNumber("kShooterD", kD);
    SmartDashboard.putNumber("kShooterFF", kFF);
    SmartDashboard.putNumber("kShooterSetPoint", setpoint);
  }

  public void tunePIDs(){
    double sdP = SmartDashboard.getNumber(("kShooterP"), 0);
    double sdI = SmartDashboard.getNumber(("kShooterI"), 0);
    double sdD = SmartDashboard.getNumber(("kShooterD"), 0);
    double sdFF = SmartDashboard.getNumber(("kShooterFF"), 0);

    if (sdP != kP){
      kP = sdP;
      topShooterPID.setP(kP);
      bottomShooterPID.setP(kP);
    } 
    if (sdI != kI){
      kI = sdI;
      topShooterPID.setP(kI);
      bottomShooterPID.setP(kI);
    }
    if (sdD != kD){
      kD = sdD;
      topShooterPID.setP(kD);
      bottomShooterPID.setP(kD);
    } 
    if (sdFF != kFF){
      kFF = sdFF;
      topShooterPID.setP(kFF);
      bottomShooterPID.setP(kFF);
    }
  }

  public void runShooterSmartDashboard(){
    setpoint = SmartDashboard.getNumber(("kShooterSetPoint"), 0);
    topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID     
  }
  public void runShooter(Orientations orientation){
    requestedSetpoint = orientation.shooterSpeed;
    setpoint = orientation.shooterSpeed;
    topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID     
  }

  public void runShooterBasedOnDistance(double velocity){
    requestedSetpoint = velocity;
    setpoint = velocity;
    topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID     
  }

  public void stopShooterSmartDashboard(){
    topShooterMotor.set(0);
  }
  public void stopShooter(Orientations orientation){
    // humSpeed is the slow maintained hum of the shooter motors during the course of the game to allow faster spinup
    if ((orientation.maintainHumSpeed) && !HumModeOff){
      // when we want to stop the shooter but maintainHumSpeed is set, we set it to the humSpeed instead
      setpoint = humSpeed;
      topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
    } else {
      // when we just want to stop the motor (HOME, or CLIMBING), stop the motor fully
      topShooterMotor.set(0);
    }
  }

  public boolean isTopShooterAtSpeed(){
    double encoderValue = topShooterEncoder.getVelocity();
    //System.out.println("-- || || || TOP SHOOTER VELOCITY: " + encoderValue);
    double tolerance = setpointTolerance;
    double topSetpoint = requestedSetpoint;
    //System.out.println("-- || || || TOP SHOOTER REQUESTED VELOCITY: " + topSetpoint);
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance + 2000 + FreakyConstants.Freaky;
    boolean isWithinTolerance = topSetpoint != 0 && encoderValue >= minLimit && encoderValue <= maxLimit;
    //System.out.println("-- || || || TOP SHOOTER isWithinTolerance: " + isWithinTolerance);
    return isWithinTolerance;
  }

  public boolean areShootersAtSpeed(){
    //System.out.println("-- || || || || ARE BOTH SHOOTERS WITHIN TOLERANCES?: " + isTopShooterAtSpeed());
    return isTopShooterAtSpeed();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("TOP Shooter RPM: ", topShooterEncoder.getVelocity());
    SmartDashboard.putNumber("TOP Shooter getPosition: ", topShooterEncoder.getPosition());
    
    

    if (TUNING_MODE){
      //final InstantCommand runShooterCmd = new InstantCommand(()->runShooterSmartDashboard());
      //final InstantCommand stopShooterCmd = new InstantCommand(()->stopShooterSmartDashboard());
    
      //SmartDashboard.putData(CommandScheduler.getInstance());
      //SmartDashboard.putData(RobotContainer.shooterSubsystem);
      //SmartDashboard.putData("Run Shooter", runShooterCmd);
      //SmartDashboard.putData("Stop Shooter", stopShooterCmd);

      addPIDToDashboard();
      tunePIDs();
    }
    
  }
}
