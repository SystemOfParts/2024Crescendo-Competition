package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeRun extends SequentialCommandGroup{
    public IntakeRun(
        IntakeSubsystem m_intake

    ){
    addCommands(
        new RunCommand(() -> m_intake.RunIntake()));
 
    }
}