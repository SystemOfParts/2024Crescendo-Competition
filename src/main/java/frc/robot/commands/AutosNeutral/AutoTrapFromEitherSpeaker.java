// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosNeutral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoShootFromDistance;
import frc.robot.commands.AutoShootFromSubwoofer;
import frc.robot.commands.CheckToShoot;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTrapFromEitherSpeaker extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  public String trajectory = "BlueSubTrap";
  public AutoTrapFromEitherSpeaker(
    ArmSubsystem m_arm,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (RobotContainer.isAlianceRed){
      trajectory = "RedSubTrap";
    } 
    addCommands(
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAP_SCORE),
        new RunTrajectorySequenceRobotAtStartPoint(trajectory)
      ),

      new AutoShootFromDistance(m_arm, m_shooter, m_intake)
      // END AUTO
    );
  }
}
