// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.OrientationConstants.Orientations;

import frc.robot.RobotContainer;


public class ShooterSubsystem extends SubsystemBase {

    public static final RobotContainer robotContainer = new RobotContainer();
    
    private final CANSparkMax bottomShooterMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax topShooterMotor = new CANSparkMax(14, MotorType.kBrushless);
    private final SparkPIDController bottomShooterPID = bottomShooterMotor.getPIDController();
    private final SparkPIDController topShooterPID = topShooterMotor.getPIDController();
    //private boolean runOnce = true;


// PID constants taken from example code
    private static final double kP = 0.0004; // Proportional term - make lower tommorow
    private static final double kI = 0.001; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 10; // Integral zone
    private static final double kFF = 0.0002; // Feed-forward
    private static final double kMaxOutput = 1; // Change these later
    private static final double kMinOutput = -1;
    private double stopSpeed = 0;
    private double humSpeed = 500;
    public double setpoint = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.clearFaults();
    bottomShooterMotor.clearFaults();


    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);

    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);

    
     //VPID configuration 
    bottomShooterPID.setP(kP);
    bottomShooterPID.setI(kI);
    bottomShooterPID.setD(kD);
    bottomShooterPID.setIZone(kIz);
    bottomShooterPID.setFF(kFF);
    
    bottomShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    bottomShooterPID.setSmartMotionMaxVelocity(5600, 0);
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

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();

    
  }

    public void runShooter(Orientations orientation){
          setpoint = orientation.shooterSpeed;
          bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
          topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID     
    }

    public void stopShooter(Orientations orientation){
      // humSpeed is the slow maintained hum of the shooter motors during the course of the game to allow faster spinup
      if (orientation.maintainHumSpeed){
        // when we want to stop the shooter but maintainHumSpeed is set, we set it to the humSpeed instead
        setpoint = humSpeed;
      } else {
        // when we just want to stop the motor (HOME, or CLIMBING), stop the motor fully
        setpoint = stopSpeed;
      }
      bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
      topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

    }


  

  @Override
  public void periodic() {

  }
}