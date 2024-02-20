package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
public class ArmMoveDown extends SequentialCommandGroup{
    public ArmMoveDown(
        ArmSubsystem m_arm

    ){
    addCommands(
        new RunCommand(() -> m_arm.armMoveDown()));
    }
}