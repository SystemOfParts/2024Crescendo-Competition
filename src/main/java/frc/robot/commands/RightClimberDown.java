package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class RightClimberDown extends SequentialCommandGroup{
    public RightClimberDown(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new InstantCommand(() -> m_climber.climber2Down(.25)));
       
    }
}