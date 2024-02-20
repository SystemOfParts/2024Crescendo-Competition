package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class LeftClimberDown extends SequentialCommandGroup{
    public LeftClimberDown(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new RunCommand(() -> m_climber.climber1Down(.5)));
    }
}