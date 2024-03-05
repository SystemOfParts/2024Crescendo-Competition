// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosRed;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.MoveToOrientationCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoOneNote;
import frc.robot.commands.CheckToShoot;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedCenterFourNote extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public AutoRedCenterFourNote(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      new AutoOneNote(m_arm, m_shooter, m_intake),
      
      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterFourNotePart1")
      ),

      // with the second note loaded, orient to podium and move to position 2
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_PODIUM),
        // this trajectory was modified slightly to stop in front of the note w/ room for the intake
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterFourNotePart2")
      ),
      
      // Make sure the shooter is still at speed
      new CheckToShoot(m_shooter, m_intake),

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)),

      // move the arm down to intake position
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
      
      // path the robot backwards through the 3rd note to pick it up with the intake
      new RunTrajectorySequenceRobotAtStartPoint("RedCenterFourNotePart3"),
      
      // with the 3rd note, orient to podium and move to position 4
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_PODIUM),
        // this trajectory was modified slightly to stop in front of the note w/ room for the intake
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterFourNotePart4")
      ),

      // make sure the shooter is up to speed
      new CheckToShoot(m_shooter, m_intake),

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)),

      // Move to intake position to get the last note
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),

      // path the robot backwards through the 4th note to pick it up with the intake
      new RunTrajectorySequenceRobotAtStartPoint("RedCenterFourNotePart5"),
      
      // Make sure the shooter is still at speed
      new CheckToShoot(m_shooter, m_intake),

      // Feed the intake to actually shoot (still using Podium speed and orientation)
      new InstantCommand(() -> m_intake.runIntake(true)));
  }
}
