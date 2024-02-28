package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedShooterCommand extends SequentialCommandGroup{
    public FeedShooterCommand(
        ShooterSubsystem m_shooter,
        IntakeSubsystem m_intake

    ){
        //if (m_shooter.areShootersAtSpeed()){
            addCommands(
                new RunCommand(() -> m_intake.runIntake()));
        //}
    }
}