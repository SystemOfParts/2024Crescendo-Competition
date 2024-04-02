// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosBlue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.commands.*;
import frc.robot.commands.AutonomousCommands.AutoMoveToOrientationCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FAST_AutoBlueSouthThreeNoteMid3 extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAuto. */
  public FAST_AutoBlueSouthThreeNoteMid3(
    ArmSubsystem m_a,
    IntakeSubsystem m_i,
    ShooterSubsystem m_s
  ) {
    addCommands(
            new WaitCommand(.5),
            new InstantCommand(() -> RobotContainer.imuSubsystem.setYaw(-60)), // set yaw to the one in the initial pose

      new InstantCommand(() -> m_s.runShooter(Orientations.AUTO_SUBWOOFER)),
      new AutoIntakeMoveShoot(false, Orientations.AUTO_SUBWOOFER, null, null, 0.5, 0.5, m_a, m_s, m_i),
      new AutoIntakeMoveShoot(true, Orientations.AUTO_PODIUM, "a_b_South_2Note_SAN", null, 0.5, 0.5, m_a, m_s, m_i),
      new AutoIntakeMoveShoot(true, Orientations.AUTO_PODIUM, "a_b_South_3Note_Mid3", null, 0.5, 0.5, m_a, m_s, m_i),
      new AutoMoveToOrientationCommand(m_a, m_s, m_i, Orientations.HOME));
  }
}