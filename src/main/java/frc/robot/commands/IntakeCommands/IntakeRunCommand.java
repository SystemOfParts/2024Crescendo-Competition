package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeRunCommand extends SequentialCommandGroup{
    public IntakeRunCommand(
        IntakeSubsystem m_intake

    ){
    addCommands(
        new RunCommand(() -> m_intake.runIntake()));
 
    }
}