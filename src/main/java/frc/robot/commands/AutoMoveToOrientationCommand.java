package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoMoveToOrientationCommand extends SequentialCommandGroup {
    public AutoMoveToOrientationCommand(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            IntakeSubsystem m_intake,
            Orientations orientation) {

       
        if ((Objects.nonNull(m_arm)) && (Objects.nonNull(m_shooter)) && (Objects.nonNull(m_intake))){
            //System.out.println("*** m_leadscrew and arm were not null");   
            addCommands(
                new ParallelCommandGroup(
                    new InstantCommand(() -> System.out.println("----->>> ORIENTING TO: " + orientation.label)),
                    new InstantCommand(() -> m_arm.moveToPosition(orientation)),
                    new InstantCommand(() -> m_arm.leadMoveToPosition(orientation)),

                    new InstantCommand(() -> System.out.println("**TURN OFF SHOOTER" + orientation.label)),
                    new InstantCommand(() -> m_shooter.stopShooter(orientation)),

                    new InstantCommand(() -> System.out.println("**TURN OFF INTAKE" + orientation.label)),

                    new InstantCommand(m_intake::stopIntake),
                    new WaitCommand(1)));
        
      
            } 
        
        if (Objects.nonNull(m_intake)){
            if (orientation.intakeOn) {
                addCommands(
                    new InstantCommand(() -> System.out.println("**TURN ON INTAKE" + orientation.label)),
                    new InstantCommand(() -> m_intake.runIntake()));
            }
        }
    }
}