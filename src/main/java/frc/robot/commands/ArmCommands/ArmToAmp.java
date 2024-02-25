package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
public class ArmToAmp extends SequentialCommandGroup{
    public ArmToAmp(
        ArmSubsystem m_arm

    ){
    addCommands(
        new InstantCommand(() -> m_arm.armToAmp()));
    }
}