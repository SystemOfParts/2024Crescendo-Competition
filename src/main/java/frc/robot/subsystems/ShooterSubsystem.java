// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.RobotContainer;


public class ShooterSubsystem extends SubsystemBase {

    public static final RobotContainer robotContainer = new RobotContainer();
    
    private final CANSparkMax bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax topShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final SparkPIDController shooterPID = bottomShooterMotor.getPIDController();

// PID constants taken from example code
    private static final double kP = 0.0004; // Proportional term - make lower tommorow
    private static final double kI = 0.001; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 0; // Integral zone
    private static final double kFF = 0.0002; // Feed-forward
    private static final double kMaxOutput = 1; // Change these later
    private static final double kMinOutput = -1;
    public double setpoint = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.follow(bottomShooterMotor, false);

    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);

    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();

     //PID configuration 
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);
    
  }

     public void runShooter(){

      bottomShooterMotor.set(.1); //5700 upper limit
      
    }
    public void stopShooter(){
      
      bottomShooterMotor.set(0);
      
    }
  

  @Override
  public void periodic() {

    /*
    controls for shooter: 


    Left trigger = subwoofer shooting
    Right Trigger = far shooting
    Right bumper = amp spit out

    */
  
    
    if (robotContainer.getLeftTrigger() > .25) {

      setpoint = 3000 * robotContainer.getLeftTrigger();

    }
    else if (robotContainer.getRightBumper()){

      setpoint = 500;

    }
    else {
    setpoint = 5700 * robotContainer.getRightTrigger();
    }
    
    shooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

    // This method will be called once per scheduler run
  }
}