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

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climberMotor1 = new CANSparkMax(Constants.CanIdConstants.kClimber1Id, MotorType.kBrushless);
  private final CANSparkMax climberMotor2 = new CANSparkMax(Constants.CanIdConstants.kClimber2Id, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder1;
  private final RelativeEncoder climberEncoder2;

  private static final double MAX_POSITION = Constants.ClimberConstants.kMAX_POSITION;//-400.0
  private static final double MIN_POSITION = Constants.ClimberConstants.kMIN_POSITION;//-10

  /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {

        climberEncoder1 = climberMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        climberEncoder2 = climberMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42); // this takes into account encoder resolution

        climberMotor1.restoreFactoryDefaults();
        climberMotor2.restoreFactoryDefaults();

        climberMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        climberMotor2.setInverted(true);

        climberMotor1.burnFlash();
        climberMotor2.burnFlash();

        climberEncoder1.setPosition(0);
        climberEncoder2.setPosition(0);
    }
    
    public void climber1Up(double speed) {
        //System.out.println("POSITION: "+(climberEncoder1.getPosition()));
        System.out.println("MAX_POSITION: "+(MAX_POSITION));
        System.out.println("POS-> "+(climberEncoder1.getPosition())+" > "+MAX_POSITION+" <-MAX");
        if (climberEncoder1.getPosition() > MAX_POSITION) {
            System.out.println("GOING UP");
            climberMotor1.set(-speed);
        } else {
            climberMotor1.set(0);
        }
    }
      public void climber2Up(double speed) {
        System.out.println("POSITION: "+(climberEncoder2.getPosition()));
        System.out.println("MAX_POSITION: "+(MAX_POSITION));
        System.out.println("POS-> "+(climberEncoder2.getPosition())+" > "+MAX_POSITION+" <-MAX");
        if (climberEncoder2.getPosition() > MAX_POSITION) {
            System.out.println("GOING UP");
            climberMotor2.set(-speed);
        } else {
            climberMotor2.set(0);
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
  public void climber2Down(double speed) {
        System.out.println("POSITION: "+(climberEncoder2.getPosition()));
        System.out.println("MIN_POSITION: "+(MIN_POSITION));
        System.out.println("POS-> "+(climberEncoder2.getPosition())+" < "+MIN_POSITION+" <-MIN");
        if (climberEncoder2.getPosition() < MIN_POSITION) {
            System.out.println("GOING DOWN");
            climberMotor2.set(speed);
        } else {
            climberMotor2.set(0);
        }
    }
    //dangerous controls for testing
  public void climber1DownManually(double speed){
    climberMotor1.set(speed);
  }
  public void climber1UpManually(double speed){
    climberMotor1.set(-speed);
  }
   public void climber2DownManually(double speed){
    climberMotor2.set(speed);
  }
  public void climber2UpManually(double speed){
    climberMotor2.set(-speed);
  }

   // Returns the climber position
    public double getClimber1Position() {
        return climberEncoder1.getPosition();
    }

    public double getClimber2Position() {
        return climberEncoder2.getPosition();
    }

  public void stopClimber1() {
    climberMotor1.set(0);
  }
  public void stopClimber2() {
    climberMotor2.set(0);
  }

 

  @Override
  public void periodic() {
    System.out.println(climberEncoder2.getPosition());
    System.out.println(climberEncoder1.getPosition());

  }
}
