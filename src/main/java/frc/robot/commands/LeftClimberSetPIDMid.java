package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class LeftClimberSetPIDMid extends SequentialCommandGroup{
    public LeftClimberSetPIDMid(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new InstantCommand(() -> m_climber.moveToMiddle()));
    }
}