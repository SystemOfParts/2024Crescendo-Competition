package frc.robot.commands.NoteDetectionCommands;

import java.time.Instant;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.commands.TurnToDegreeIMU;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NoteDetectionPHTNVisionSubsystem;

public class OrientToNoteCommand extends Command{
  private final DoubleSupplier m_triggerValue;
  private NoteDetectionPHTNVisionSubsystem phtn;
  private boolean intendedDetection = false;
  private boolean foundAngle = false;


  public OrientToNoteCommand(DoubleSupplier triggerValue) 
  {
    m_triggerValue = triggerValue;
    addRequirements(RobotContainer.noteDetectionPhtnVisionSubsystem);
  }

  @Override
  public void initialize() {
    foundAngle = false;
    phtn = RobotContainer.noteDetectionPhtnVisionSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intendedDetection = false;
    if (m_triggerValue.getAsDouble() > .1){
      if (!foundAngle){
        phtn.getPHTNData();
        //System.out.println("::::::IS NOTE VISIBLE?: "+phtn.isNoteVisible());
        if (phtn.isNoteVisible()){
          foundAngle = true;
          intendedDetection = true;
          // UNCOMMENT TO TEST ROTATING TO THE ANGLE
          //System.out.println("::::::::::NOTE YAW: "+phtn.getNoteYaw());
          //System.out.println("::::::::::ROBOT YAW: "+RobotContainer.imuSubsystem.getYaw());
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("::::::::::ORIENT TO NOTE ENDED WAS IT INTERRUPTED?: "+interrupted);
 
      
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}