// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class XboxRumbleCommand extends SequentialCommandGroup {
  /** Creates a new XboxRumbleCommand. */
  public XboxRumbleCommand(double seconds, double intensity, RumbleType rumbleType) {
    addCommands(
      new InstantCommand( () -> RobotContainer.xboxController.setRumble(rumbleType, intensity) ), // start rumble half-intensity
      new WaitCommand(seconds),
      new InstantCommand( () -> RobotContainer.xboxController.setRumble(rumbleType, 0) ) // stop rumble
    );
  }
}