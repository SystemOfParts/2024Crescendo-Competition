// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.*;

public class AutoDynamicStepCommand extends SequentialCommandGroup {
  public AutoDynamicStepCommand( 
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      IntakeSubsystem m_intake,
      Boolean hasIntake, 
      Boolean hasPrePath,
      Boolean hasPostPath,
      Orientations shotOrientation,
      String prePath,
      String postPath
      )
    {
    // IF we have a path to travel before we intake
    if (hasPrePath){
      // There was a pre path
      if (hasIntake){
        // There is an intake task
      addCommands(
        // together orient to the intake position and start moving 
        new ParallelCommandGroup(
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
          new RunTrajectorySequenceRobotAtStartPoint(prePath)));
      }
    } else {
      // There is no intake task
      addCommands(
        // just move where we're supposed to go before shooting
        new RunTrajectorySequenceRobotAtStartPoint(prePath));
    }
    // IF we have a note
    if (m_intake.isNoteInIntake()){
      // shoot the note using the current intake
      addCommands(
        new AutoShootAtCurrentTarget(m_arm, m_shooter, m_intake));
    }
    if (hasPostPath){
      addCommands(
        new RunTrajectorySequenceRobotAtStartPoint(postPath));
    }
    addCommands(
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL)
    );
  }
}
