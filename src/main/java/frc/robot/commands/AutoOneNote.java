// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeStopCommand;
import frc.robot.commands.CheckToShoot;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoOneNote extends SequentialCommandGroup {
  public AutoOneNote( 
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      IntakeSubsystem m_intake
      )
    {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.SUBWOOFER),
      new CheckToShoot(m_shooter, m_intake),
      new InstantCommand(() -> m_intake.runIntake()),
      // only wait a half of a second to make sure the note has been fired
      new WaitCommand(.5)
      // Leave the intake on so we can move quickly start moving to INTAKE position
      // new IntakeStopCommand(m_intake)
      // why is this here?
      {}
      );
  }
}
