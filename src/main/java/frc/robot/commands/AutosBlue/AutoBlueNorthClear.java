// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosBlue;
import frc.robot.commands.AutoMoveToOrientationCommand;
import frc.robot.commands.AutoShootFromSubwoofer;
import frc.robot.commands.CheckToShoot;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoBlueNorthClear extends SequentialCommandGroup {
  public AutoBlueNorthClear( 
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      IntakeSubsystem m_intake
      )
    {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootFromSubwoofer(m_arm, m_shooter, m_intake),
      
      // with the shooter and intake running, orient arm to the intake position AND starting to move to pick up the 2nd note
      new ParallelCommandGroup(
        new AutoMoveToOrientationCommand(m_arm, m_shooter, m_intake, Orientations.TRAVEL),
        // this trajectory was modified slightly to move through the note to intake it
        new RunTrajectorySequenceRobotAtStartPoint("BlueNorthCenterClear")
      )
    );
  }
}
