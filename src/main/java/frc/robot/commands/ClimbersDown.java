package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersDown extends SequentialCommandGroup{
    public ClimbersDown(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new InstantCommand(() -> m_climber.climber1Down(.25)),
        new InstantCommand(() -> m_climber.climber2Down(.25)));
    }
}