// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.MoveToOrientationCommand;
import frc.robot.commands.OneNoteAuto;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAuto extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public ThreeNoteAuto(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new OneNoteAuto(m_arm, m_shooter, m_intake),
    
    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),

    new RunTrajectorySequenceRobotAtStartPoint("5142_ThreeNotePart1"),

    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.PODIUM),

    new CheckToShoot(m_shooter, m_intake),

    new InstantCommand(() -> m_intake.runIntake()),

    new WaitCommand(1),

    new IntakeStopCommand(m_intake),

    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.AUTO_INTAKE),

    new RunTrajectorySequenceRobotAtStartPoint("5142_ThreeNotePart2"),

    new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.PODIUM),

    new CheckToShoot(m_shooter, m_intake),

    new InstantCommand(() -> m_intake.runIntake()),

    new WaitCommand(1),

    new IntakeStopCommand(m_intake)


    );
  }
}
