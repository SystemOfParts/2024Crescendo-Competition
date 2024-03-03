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
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.OrientationConstants.Orientations;

import frc.robot.RobotContainer;


public class ShooterSubsystem extends SubsystemBase {

  private boolean TUNING_MODE = false;

  private final CANSparkMax bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax topShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
  
  private RelativeEncoder bottomShooterEncoder;
  private RelativeEncoder topShooterEncoder;
  
  private final SparkPIDController bottomShooterPID = bottomShooterMotor.getPIDController();
  private final SparkPIDController topShooterPID = topShooterMotor.getPIDController();
  //private boolean runOnce = true;

// PID constant for tuning
  double kP = 0.0004; // Proportional term - make lower tommorow
  double kI = 0.001; // Integral term
  double kD = 0.0; // Derivative term
  double kIz = 10; // Integral zone
  double kFF = 0.0002; // Feed-forward
  double kMaxOutput = 1; // Change these later
  double kMinOutput = -1;
  //private double stopSpeed = 0;
  double requestedSetpoint = 0;
  double humSpeed = 500;
  double setpoint = 0;
  double setpointTolerance = 100;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.clearFaults();
    bottomShooterMotor.clearFaults();

    topShooterEncoder = topShooterMotor.getEncoder();
    bottomShooterEncoder = bottomShooterMotor.getEncoder();

    topShooterPID.setFeedbackDevice(topShooterEncoder);
    bottomShooterPID.setFeedbackDevice(bottomShooterEncoder);

     //VPID configuration 
    bottomShooterPID.setP(kP);
    bottomShooterPID.setI(kI);
    bottomShooterPID.setD(kD);
    bottomShooterPID.setIZone(kIz);
    bottomShooterPID.setFF(kFF);
    
    bottomShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    bottomShooterPID.setSmartMotionMaxVelocity(3000, 0);
    bottomShooterPID.setSmartMotionMinOutputVelocity(500, 0);
    bottomShooterPID.setSmartMotionMaxAccel(3000, 0);
    bottomShooterPID.setSmartMotionAllowedClosedLoopError(50, 0);

    topShooterPID.setP(kP);
    topShooterPID.setI(kI);
    topShooterPID.setD(kD);
    topShooterPID.setIZone(kIz);
    topShooterPID.setFF(kFF);
    
    topShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    topShooterPID.setSmartMotionMaxVelocity(5600, 0);
    topShooterPID.setSmartMotionMinOutputVelocity(500, 0);
    topShooterPID.setSmartMotionMaxAccel(3000, 0);
    topShooterPID.setSmartMotionAllowedClosedLoopError(50, 0);

    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);

    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();

    if (TUNING_MODE){addPIDToDashboard();}
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

  public void runShooter(Orientations orientation){
    requestedSetpoint = orientation.shooterSpeed;
    setpoint = orientation.shooterSpeed;
    bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
    topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID     
  }

  public void stopShooter(Orientations orientation){
    // humSpeed is the slow maintained hum of the shooter motors during the course of the game to allow faster spinup
    if (orientation.maintainHumSpeed){
      // when we want to stop the shooter but maintainHumSpeed is set, we set it to the humSpeed instead
      setpoint = humSpeed;
      bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
      topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
    } else {
      // when we just want to stop the motor (HOME, or CLIMBING), stop the motor fully
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
    }
  }

  public boolean isTopShooterAtSpeed(){
    double encoderValue = topShooterEncoder.getVelocity();
   // System.out.println("-- || || || TOP SHOOTER VELOCITY: " + encoderValue);
    double tolerance = setpointTolerance;
    double topSetpoint = requestedSetpoint;
    //System.out.println("-- || || || TOP SHOOTER REQUESTED VELOCITY: " + topSetpoint);
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance;
    boolean isWithinTolerance = topSetpoint != 0 && encoderValue >= minLimit && encoderValue <= maxLimit;
    //System.out.println("-- || || || TOP SHOOTER isWithinTolerance: " + isWithinTolerance);
    return isWithinTolerance;
  }

  public boolean isBottomShooterAtSpeed(){
    double encoderValue = bottomShooterEncoder.getVelocity();
    //System.out.println("-- || || || BOTTOM SHOOTER VELOCITY: " + encoderValue);
    double tolerance = setpointTolerance;
    double bottomSetpoint = requestedSetpoint;
    //System.out.println("-- || || || BOTTOM SHOOTER REQUESTED VELOCITY: " + bottomSetpoint);
    double minLimit = setpoint - tolerance;
    double maxLimit = setpoint + tolerance;
    boolean isWithinTolerance = bottomSetpoint != 0 && encoderValue >= minLimit && encoderValue <= maxLimit;
   // System.out.println("-- || || || BOTTOM SHOOTER isWithinTolerance: " + isWithinTolerance);
    return isWithinTolerance;
  }

  public boolean areShootersAtSpeed(){
    //System.out.println("-- || || CHECKING SHOOTERS ARE WITHIN "+setpointTolerance+" OF SPEED || || --");
    boolean bothWithinTolerances = (isTopShooterAtSpeed() && isBottomShooterAtSpeed());
    //System.out.println("-- || || || || ARE BOTH SHOOTERS WITHIN TOLERANCES?: " + bothWithinTolerances);
    return bothWithinTolerances;
  }

  @Override
  public void periodic() {

  }
}