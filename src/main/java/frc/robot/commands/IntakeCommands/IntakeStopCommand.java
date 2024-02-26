package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeStopCommand extends SequentialCommandGroup{
    public IntakeStopCommand(
        IntakeSubsystem m_intake

    ){
    addCommands(
        new InstantCommand(() -> m_intake.stopIntake()));
 
    }
}