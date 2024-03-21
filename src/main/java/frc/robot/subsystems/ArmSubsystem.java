// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.OrientationConstants.Orientations;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;

public class ArmSubsystem extends SubsystemBase {

    private static final String printLocation = "ArmSubsystem: ";
    private boolean CONTROL_MANUALLY = false;
    // declare the arm motors
    private final static CANSparkMax leftArmMotor = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax rightArmMotor = new CANSparkMax(11, MotorType.kBrushless);

    // declare encoders for both
    public final static RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();

    
    // declare PID Controllers for both 
    private final SparkPIDController armController = leftArmMotor.getPIDController();


    double ArmkP = 0.1; // Proportional term
    double ArmkI = 0.0; // Integral term
    double ArmkD = 0.01; // Derivative term
    double ArmkIz = 0; // Integral zone
    double ArmkFF = 0.0; // Feed-forward
    double ArmkMaxOutput = .6;
    double ArmkMinOutput = -.5;


    public float kArmUpLimit = 52;
    public float kArmDownLimit = 0;

    public double armSetpoint = 0;

    // Gear reduction and encoder conversion factor - NEEDS UPDATE
    //private static final double gearReduction = 197.142857143; // 197.142857143:1 Gear reduction (Not taking chain & Sprocket into account)
    //private static final double encoderConversionFactor = 360.0 / gearReduction;
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
        


    }

    public void moveToPosition(Orientations orientation) {
        setArmPosition(orientation.armPosition);
    }

    public void setArmPosition(double armPosition) {
      //System.out.println("**ARM TRYING TO MOVE TO" + armPosition);
        // move to the position dynamically
        // need method to VERIFY armPosition is SAFE (within bounds) before using!!!!
        armSetpoint = armPosition;
    }

    public void manualModeTurnOn() {
      //System.out.println("Turned on climber mode");
    
      CONTROL_MANUALLY = true;
    }
    
    
    public void manualModeTurnOff() {
      //System.out.println("Turned off climber mode");
    
      CONTROL_MANUALLY = false;
    }
    
    public boolean getManualMode() {
    
      //System.out.println("Accessed Climbing Mode");
      
      return CONTROL_MANUALLY;
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.print(encoder.getPosition());
    //System.out.println("encoder position: ");
    
    armController.setReference(armSetpoint, CANSparkMax.ControlType.kPosition); //applies the chosen PID
  }
}