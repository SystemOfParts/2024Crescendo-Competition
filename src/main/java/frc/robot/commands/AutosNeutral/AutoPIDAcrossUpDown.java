// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosNeutral;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoPIDAcrossUpDown extends SequentialCommandGroup {
  public AutoPIDAcrossUpDown(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    addCommands(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross")
    );
  }
}
