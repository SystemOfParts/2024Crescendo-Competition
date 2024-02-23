// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax topShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax bottomShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final SparkPIDController shooterPID = bottomShooterMotor.getPIDController();

// PID constants taken from example code
    private static final double kP = 6e-5; // Proportional term 
    private static final double kI = 0.0; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 0; // Integral zone
    private static final double kFF = 0.000015; // Feed-forward
    private static final double kMaxOutput = .25; // Change these later
    private static final double kMinOutput = -.25;
    public double setpoint = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooterMotor.follow(bottomShooterMotor, true);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);

    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();

    // PID configuration
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);
  }

     public void runShooter(){

      setpoint = 1000; //5700 upper limit
      
    }
    public void stopShooter(){
      
      setpoint = 0;
      
    }
  

  @Override
  public void periodic() {

    shooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

    // This method will be called once per scheduler run
  }
}

