// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.AutonomousCommands.AutoShootAtCurrentTarget;

public class AutoStepDynCmd extends SequentialCommandGroup {
  public AutoStepDynCmd( 
      Boolean hasIntake,
      Orientations shotOrientation,
      String prePath,
      String postPath,
      Double preDelay,
      Double postDelay,
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      IntakeSubsystem m_intake
      )
    {
    Orientations lastOrientation = null;
    // IF we have a path to travel before we intake
    if (prePath != null){
      // There was a pre path
      if (hasIntake){
        // There is an intake task
      lastOrientation = Orientations.AUTO_INTAKE;
      addCommands(
        // together orient to the intake position and start moving 
        new ParallelCommandGroup(
          new FAST_AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
          new FASTRunTrajectorySequenceRobotAtStartPoint(prePath)));
      }
    } 
    //PREPATH COMPLETED OR SKIPPED, WE'VE STOPPED MOVING, HOPEFULLY WE HAVE A NOTE
    // LET'S CHECK FOR A NOTE
    System.out.println("FAST AUTO : PRE FINISHED, IS NOTE IN INTAKE?: "+m_intake.isNoteInIntake());
    if (m_intake.isNoteInIntake()){
      // WE HAVE A NOTE
      // ARE WE SUPPOSED TO MOVE TO A SHOOTING POSITION?
      if (postPath != null){
        // YES - LETS ORIENT AND MOVE TOGETHER
        addCommands(
          new ParallelCommandGroup(
            new FAST_AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, shotOrientation),
            new FASTRunTrajectorySequenceRobotAtStartPoint(postPath)));
      } else {
        // WE DON'T NEED TO MOVE, LET'S JUST ORIENT TO SHOOTING POSITION
        System.out.println("FAST AUTO ORIENT TO SHOOTING POSITION WITHOUT POST MOVEMENT");
        addCommands(
          new FAST_AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, shotOrientation));
      }
      // WE HAVE FINISHED ORIENTING AND MOVING AS NEEDED LETS USE THIS NOTE
      System.out.println("SHOOTING WITH preDelay of: "+preDelay+" and postDelay of: "+postDelay);
      
    }
      addCommands(
        new WaitCommand(preDelay),
        new AutoShootAtCurrentTarget(m_arm, m_shooter, m_intake),
        new WaitCommand(postDelay));
  }
}
