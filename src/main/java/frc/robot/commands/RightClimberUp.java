package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class RightClimberUp extends SequentialCommandGroup{
    public RightClimberUp(
        ClimberSubsystem m_climber
    ){
    addCommands(
        new RunCommand(() -> m_climber.climber2Up(.5)));
    }
}