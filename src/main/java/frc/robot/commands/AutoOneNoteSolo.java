// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoOneNoteSolo extends SequentialCommandGroup {
  public AutoOneNoteSolo( 
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      IntakeSubsystem m_intake
      )
    {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.SUBWOOFER),
      new WaitCommand(2.5),
      new CheckToShoot(m_shooter, m_intake),
      new InstantCommand(() -> m_intake.runIntake(true)),
      new WaitCommand(1),
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL)
    );
  }
}
