// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDChangePatternCommand extends SequentialCommandGroup {
  public LEDChangePatternCommand( LEDSubsystem LEDs, BlinkinPattern pattern)
    {
    addCommands(
      new InstantCommand(() -> LEDs.setPattern(pattern))
    );
  }
}
