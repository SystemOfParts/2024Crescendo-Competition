// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutosNeutral;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoPIDAcrossUpDown extends SequentialCommandGroup {
  public AutoPIDAcrossUpDown( 

      )
    {
    addCommands(
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down"),
      new WaitCommand(5),
      new RunTrajectorySequenceRobotAtStartPoint("PIDAcross-Up-Down")
    );
  }
}
