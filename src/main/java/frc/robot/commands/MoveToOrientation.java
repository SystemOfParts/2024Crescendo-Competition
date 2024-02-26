package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LeadScrewSubsystem;

public class MoveToOrientation extends SequentialCommandGroup {
    public MoveToOrientation(
            ArmSubsystem m_arm,
            LeadScrewSubsystem m_leadScrew,
            ShooterSubsystem m_shooter,
            IntakeSubsystem m_intake,
            Orientations orientation) {

        if (Objects.nonNull(m_leadScrew)&&Objects.nonNull(m_arm)){
            //System.out.println("*** m_leadscrew and arm were not null");   
            addCommands(
                new ParallelCommandGroup(
                    new InstantCommand(() -> System.out.println("----->>> ORIENTING TO: " + orientation.label)),
                    new InstantCommand(() -> m_arm.moveToPosition(orientation)),
                    new InstantCommand(() -> m_leadScrew.moveToPosition(orientation))));
        }
        if (Objects.nonNull(m_shooter)){
            if (orientation.shooterOn) {
                    addCommands(
                        new InstantCommand(() -> System.out.println("**TURN ON SHOOTER" + orientation.label)));
                        //new InstantCommand(m_shooter::runShooter));
            } else {
                addCommands(
                    new InstantCommand(() -> System.out.println("**TURN OFF SHOOTER" + orientation.label)));
                    //new InstantCommand(m_shooter::stopShooter));
            }
        }
        if (Objects.nonNull(m_intake)){
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
}