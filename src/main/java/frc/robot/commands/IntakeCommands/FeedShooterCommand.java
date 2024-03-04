package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedShooterCommand extends SequentialCommandGroup{
    public FeedShooterCommand(
        IntakeSubsystem m_intake
       // ShooterSubsystem m_shooter

    ){
       // if (m_shooter.areShootersAtSpeed()){
            addCommands(
                new InstantCommand(() -> m_intake.runIntake(true)));
       // }
    }
}