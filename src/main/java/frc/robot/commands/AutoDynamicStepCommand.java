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
    Orientations lastOrientation = null;
    // IF we have a path to travel before we intake
    if (hasPrePath){
      // There was a pre path
      if (hasIntake){
        // There is an intake task
      lastOrientation = Orientations.AUTO_INTAKE;
      addCommands(
        // together orient to the intake position and start moving 
        new ParallelCommandGroup(
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
          new RunTrajectorySequenceRobotAtStartPoint(prePath)));
      }
    } 
    //PREPATH COMPLETED OR SKIPPED, WE'VE STOPPED MOVING, HOPEFULLY WE HAVE A NOTE
    // LET'S CHECK FOR A NOTE
    if (m_intake.isNoteInIntake()){
      // WE HAVE A NOTE
      // ARE WE SUPPOSED TO MOVE TO A SHOOTING POSITION?
      if (hasPostPath){
        // YES - LETS ORIENT AND MOVE TOGETHER
        addCommands(
          new ParallelCommandGroup(
            new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, shotOrientation),
            new RunTrajectorySequenceRobotAtStartPoint(postPath)));
      } else {
        // WE DON'T NEED TO MOVE, LET'S JUST ORIENT TO SHOOTING POSITION
        addCommands(
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, shotOrientation));
      }
      // WE HAVE FINISHED ORIENTING AND MOVING AS NEEDED LETS USE THIS NOTE
      addCommands(
        new AutoShootAtCurrentTarget(m_arm, m_shooter, m_intake));
    }
  }
}
