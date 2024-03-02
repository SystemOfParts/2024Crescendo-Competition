// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {

  }

  /**
   * If you turn a wheel counnter-clockwise the angle value and SI value should increase
   */
  public void updateOdometryTelemetry() {
    for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Angle Encoder", RobotContainer.driveSubsystem.telemetryAngleEncoder(i));
      SmartDashboard.putNumber("S"+i+" Angle Encoder SI Corrected ", RobotContainer.driveSubsystem.telemetryAngleEncoderSI(i));
      SmartDashboard.putNumber("S"+i+" Drive Encoder", RobotContainer.driveSubsystem.telemetryDriveEncoder(i));
      
    }
    
  }

  public void updateSwerveModuleTelemetry() {

     for (int i =0; i<4; i++){
      SmartDashboard.putNumber("S"+i+" Actual Angle Encoder ", RobotContainer.driveSubsystem.actualAngleEncoder(i));
      SmartDashboard.putNumber("S"+i+" Actual Angle Encoder SI ", RobotContainer.driveSubsystem.actualAngleEncoderSI(i));

     }

  }

  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.imuSubsystem.getYaw());
    SmartDashboard.putNumber("IMU Pitch", RobotContainer.imuSubsystem.getPitch());
    SmartDashboard.putNumber("IMU Roll", RobotContainer.imuSubsystem.getRoll());

  }


  public void updateAllDisplays(){
    updateOdometryTelemetry();
    updateSwerveModuleTelemetry();
    updateIMUTelemetry();

    SmartDashboard.putNumber("Arm Encoder:", ArmSubsystem.encoder.getPosition());
    SmartDashboard.putNumber("AprilTagZ Angle (In Radians):", PHTNVisionSubsystem.getAprilTagZ());
    SmartDashboard.putNumber("Lead Screw Encoder:", ArmSubsystem.leadScrewEncoder.getPosition());

    SmartDashboard.putNumber("April Z Angle In Radians", RobotContainer.phtnVisionSubsystem.getAprilTagZAngle());

    SmartDashboard.putBoolean("April Visibility", RobotContainer.phtnVisionSubsystem.isApriltagVisible());

    SmartDashboard.putNumber("April X in M", RobotContainer.phtnVisionSubsystem.getAprilTagX());

    SmartDashboard.putNumber("April Y in M", RobotContainer.phtnVisionSubsystem.getAprilTagY());

//    SmartDashboard.putString("PV Robot Pose", RobotContainer.phtnVisionSubsystem.getRobotFieldPosePV().toString());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
