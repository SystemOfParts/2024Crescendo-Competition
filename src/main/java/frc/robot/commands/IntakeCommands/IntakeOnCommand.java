package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
public class IntakeOnCommand extends SequentialCommandGroup{
    public IntakeOnCommand(IntakeSubsystem m_intake){
                if (m_intake != null){

    addCommands(
        // runs the intake until it detects a note
        new RunCommand(() -> m_intake.runIntake(true)));
    }
}}