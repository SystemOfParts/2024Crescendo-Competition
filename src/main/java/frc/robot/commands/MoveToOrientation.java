package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveToOrientation extends SequentialCommandGroup {
    public MoveToOrientation(
            ArmSubsystem m_arm,
            //LeadScrewSubsystem m_leadScrew,
            ShooterSubsystem m_shooter,
            IntakeSubsystem m_intake,
            Orientations orientation) {

        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(() -> System.out.println("**ORIENTING TO" + orientation.label)),
                new InstantCommand(() -> m_arm.moveToPosition(orientation))));
                //new InstantCommand(() -> m_leadScrew.moveToPosition(orientation))));
        
        if (orientation.shooterOn) {
            addCommands(
                new InstantCommand(() -> System.out.println("**TURN ON SHOOTER" + orientation.label)));
                //new InstantCommand(m_shooter::runShooter));
        } else {
            addCommands(
                new InstantCommand(() -> System.out.println("**TURN OFF SHOOTER" + orientation.label)));
                //new InstantCommand(m_shooter::stopShooter));
        }

        if (orientation.intakeOn) {
            addCommands(
                new InstantCommand(() -> System.out.println("**TURN ON INTAKE" + orientation.label)));
                //new InstantCommand(m_intake::runIntake));
        } else {
            addCommands(
                new InstantCommand(() -> System.out.println("**TURN OFF INTAKE" + orientation.label)));
                //new InstantCommand(m_intake::stopIntake));
        }
    }
}