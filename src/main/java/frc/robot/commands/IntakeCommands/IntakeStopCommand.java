package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeStopCommand extends SequentialCommandGroup{
    public IntakeStopCommand(
        IntakeSubsystem m_intake, Boolean delayStop

    ){
    /* System.out.println("************************************************ INTAKE STOP COMMAND  ***"+delayStop);
    Double waitLength = 0.0;
    if (delayStop){
        System.out.println("************************************************DELAY STOP  ***"+delayStop);
        waitLength = 1.0;
    } */
    addCommands(
        //new WaitCommand(waitLength),
        new InstantCommand(() -> m_intake.stopIntake(false)));
    }
}