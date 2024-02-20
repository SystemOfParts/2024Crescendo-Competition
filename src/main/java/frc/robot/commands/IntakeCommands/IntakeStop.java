package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeStop extends SequentialCommandGroup{
    public IntakeStop(
        IntakeSubsystem m_intake

    ){
    addCommands(
        new InstantCommand(() -> m_intake.stopIntake()));
 
    }
}