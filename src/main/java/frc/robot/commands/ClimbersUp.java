package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersUp extends SequentialCommandGroup{
    public ClimbersUp(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new InstantCommand(() -> m_climber.climber1Up(.25)),
        new InstantCommand(() -> m_climber.climber2Up(.25)));
    }
}