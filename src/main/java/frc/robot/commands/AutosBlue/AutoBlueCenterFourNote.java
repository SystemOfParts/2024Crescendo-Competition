// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosBlue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoShootFromSubwoofer;
import frc.robot.commands.AutoShootFromDistance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueCenterFourNote extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public AutoBlueCenterFourNote(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    addCommands(


    // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
    new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      
    // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
    new ParallelCommandGroup(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
      new RunTrajectorySequenceRobotAtStartPoint("BlueCenterFourNotePart1")
    ),

    // with the second note loaded, orient to podium and move to position 2
    new ParallelCommandGroup(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_STARTLINE),
      new RunTrajectorySequenceRobotAtStartPoint("BlueCenterFourNotePart2")
    ),

    // with the shooter running, the intake off, and a note loaded, orient arm to the AUTO_PODIUM position 
    new AutoShootFromDistance(m_arm, m_shooter, m_intake),

    // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 3rd note
    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
    new RunTrajectorySequenceRobotAtStartPoint("BlueCenterFourNotePart3"),

    // with the second note loaded, orient to podium and move to position 3
    new ParallelCommandGroup(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_STARTLINE),
      // this trajectory was modified slightly to stop in front of the note w/ room for the intake
      new RunTrajectorySequenceRobotAtStartPoint("BlueCenterFourNotePart4")
    ),

    // move back to PODIUM orientation w/ shooter 
    new AutoShootFromDistance(m_arm, m_shooter, m_intake),

    // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 4th note
    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
    new RunTrajectorySequenceRobotAtStartPoint("BlueCenterFourNotePart5"),
    // move back to PODIUM orientation w/ shooter 
    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_PODIUM),
    // shoot
    new AutoShootFromDistance(m_arm, m_shooter, m_intake),

    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL)
    // END AUTO
    );
  }
}
