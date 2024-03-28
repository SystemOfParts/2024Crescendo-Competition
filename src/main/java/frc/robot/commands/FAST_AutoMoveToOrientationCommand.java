package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OrientationConstants.Orientations;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
import frc.robot.subsystems.ShooterSubsystem;

public class FAST_AutoMoveToOrientationCommand extends SequentialCommandGroup {
    public FAST_AutoMoveToOrientationCommand(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            IntakeSubsystem m_intake,
            Orientations auto_orientation) {
        if (Objects.nonNull(m_shooter)){
            if (auto_orientation.shooterOn) {
                    addCommands(
                        ////new InstantCommand(() -> System.out.println("**CHANGING SHOOTER SPEED TO" + auto_orientation.label)),
                        new InstantCommand(() -> m_shooter.runShooter(auto_orientation)));
            } 
        }
        if ((Objects.nonNull(m_arm)) && (Objects.nonNull(m_shooter)) && (Objects.nonNull(m_intake))){
            addCommands(
                new ParallelCommandGroup(
                    //new InstantCommand(() -> System.out.println("----->>> ORIENTING TO: " + auto_orientation.label)),
                    new InstantCommand(() -> m_arm.moveToPosition(auto_orientation))));
        }
        if (Objects.nonNull(m_intake)){
            if (auto_orientation.intakeOn) { // NEED TO STOP INTAKE WHEN NOTE DETECTION IS ADDED
                addCommands(
                    //new InstantCommand(() -> System.out.println("**TURN ON INTAKE" + auto_orientation.label)),
                    new InstantCommand(() -> RobotContainer.LEDs.setPattern(BlinkinPattern.LAWN_GREEN)),            
                    new InstantCommand(() -> m_intake.runIntake(false)));
            }
        }
    }
}