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
    private final SparkPIDController topShooterPID = bottomShooterMotor.getPIDController();


// PID constants taken from example code
    private static final double kP = 0.0004; // Proportional term - make lower tommorow
    private static final double kI = 0.001; // Integral term
    private static final double kD = 0.0; // Derivative term
    private static final double kIz = 10; // Integral zone
    private static final double kFF = 0.0002; // Feed-forward
    private static final double kMaxOutput = 1; // Change these later
    private static final double kMinOutput = -1;
    public double setpoint = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();


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
        if (orientation.label == Orientations.SUBWOOFER.label){

          setpoint = 3000;

          bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
          topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

        }
        else if (orientation.label == Orientations.AMP.label){

          setpoint = 500;
          bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
          topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

        }
        else if (orientation.label == Orientations.PODIUM.label){

          setpoint = 5500;
          bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
          topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID


        }


      
    }
    public void stopShooter(){
      
      setpoint = 0;     
      bottomShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID
      topShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity); //applies the chosen PID

    }
  

  @Override
  public void periodic() {

    /*
    controls for shooter: 
    Left trigger = subwoofer shooting
    Right Trigger = far shooting
    Right bumper = amp spit out

    /* 
    if (robotContainer.getLeftTrigger() > .25) {
      setpoint = 3000 * robotContainer.getLeftTrigger();
    } else {
      setpoint = 5700 * robotContainer.getRightTrigger();
    }    
    */

    // This method will be called once per scheduler run
  }
}