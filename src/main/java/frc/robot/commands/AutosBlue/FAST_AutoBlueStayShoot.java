// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosBlue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.AutonomousCommands.AutoMoveToOrientationCommand;
import frc.robot.commands.IntakeMoveShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FAST_AutoBlueStayShoot extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public FAST_AutoBlueStayShoot(
    ArmSubsystem m_a,
    IntakeSubsystem m_i,
    ShooterSubsystem m_s
  ) {
    addCommands(
      new InstantCommand(() -> m_s.runShooter(Orientations.AUTO_SUBWOOFER)),
      new IntakeMoveShoot(false, Orientations.AUTO_SUBWOOFER, null, null, 0.5, 0.5, m_a, m_s, m_i),
      new AutoMoveToOrientationCommand(m_a, m_s, m_i, Orientations.HOME));
  }
}