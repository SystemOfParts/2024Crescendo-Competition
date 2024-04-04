// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosRed.Archive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.RobotContainer;
import frc.robot.commands.AutonomousCommands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutonomousCommands.AutoShootFromSubwoofer;
import frc.robot.commands.PathingCommands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRedCenterMid2ToSubThreeNote extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  public AutoRedCenterMid2ToSubThreeNote(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(180)),
      // Turn on the shooter, orient to SUBWOOFER, check that shooter is at speed, feed intake to shoot, wait .5 seconds
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      
      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      
      new ParallelCommandGroup(
        //start moving WHILE we move to intake (after a 5 second pause)
        // begin moving to the next note
        new RunTrajectorySequenceRobotAtStartPoint("RedCenterThreeToMid2AndSub"),
        new SequentialCommandGroup(
          // wait half a second then put the intake down
          new WaitCommand(.5),
          new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE)
        )    
      ),
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),
      // path the robot backwards through the 3rd note to pick it up with the intake
      new RunTrajectorySequenceRobotAtStartPoint("RedCenterThreeNotePart1"),
      new RunTrajectorySequenceRobotAtStartPoint("RedCenterTwoNotePart2"),

      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      new WaitCommand(.5),
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL));
      
      // END AUTO
  }
}
