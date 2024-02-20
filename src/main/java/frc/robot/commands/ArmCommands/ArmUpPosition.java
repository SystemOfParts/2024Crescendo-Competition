package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
public class ArmUpPosition extends SequentialCommandGroup{
    public ArmUpPosition(
        ArmSubsystem m_arm

    ){
    addCommands(
        new InstantCommand(() -> m_arm.armTo85Degrees()));
    }
}