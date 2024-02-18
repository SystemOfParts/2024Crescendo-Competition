package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersDown extends ParallelCommandGroup{
    public ClimbersDown(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new RunCommand(() -> m_climber.climber1Down(.5)),
        new RunCommand(() -> m_climber.climber2Down(.5)));
    }
}