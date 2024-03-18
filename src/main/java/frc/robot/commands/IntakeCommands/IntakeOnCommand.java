package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeOnCommand extends SequentialCommandGroup{
    public IntakeOnCommand(
        IntakeSubsystem m_intake

    ){
    addCommands(
        // runs the intake until it detects a note
        new RunCommand(() -> m_intake.runIntake(false)));
    }
}