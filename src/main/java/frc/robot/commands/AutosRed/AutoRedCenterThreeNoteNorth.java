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
import frc.robot.RobotContainer;
import frc.robot.commands.MoveToOrientationCommand;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoShootAtCurrentTarget;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoShootFromSubwoofer;
import frc.robot.commands.CheckToShoot;
import frc.robot.commands.AutonomousCommands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutonomousCommands.AutoShootFromSubwoofer;
import frc.robot.commands.GPMCommands.CheckToShoot;
import frc.robot.commands.GPMCommands.MoveToOrientationCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.PathingCommands.RunTrajectorySequenceRobotAtStartPoint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedCenterThreeNoteNorth extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public AutoRedCenterThreeNoteNorth(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(180)),
      // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      new WaitCommand(.3),
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      
      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
        // this trajectory was modified slightly to move through the note to intake it
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterThreeNotePart1")
      ),
      new ParallelCommandGroup(
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterTwoNotePart2"),
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.SUBWOOFER)
      ),
      // with the shooter running, the intake off, and a note loaded, orient arm to the AUTO_PODIUM position 
      new ParallelCommandGroup(
        new WaitCommand(.3),
        new AutoShootAtCurrentTarget(m_arm, m_shooter, m_intake)),

      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
        // path the robot backwards through the 3rd note to pick it up with the intake
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterThreeNotePart3CompletePod"),
        new SequentialCommandGroup(
          new WaitCommand(2.5),
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.PODIUM)
        )
      ),
      // move back to SUBWOOFER orientation w/ shooter 
      new AutoShootAtCurrentTarget(m_arm, m_shooter, m_intake),
      new WaitCommand(.3),
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.HOME));
      // END AUTO
  }
}
