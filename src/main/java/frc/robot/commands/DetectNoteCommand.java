package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NoteDetectionPHTNVisionSubsystem;
public class DetectNoteCommand extends Command{
  private final DoubleSupplier m_triggerValue;
  private NoteDetectionPHTNVisionSubsystem phtn;
  private boolean recentDetection = false;


  public DetectNoteCommand(DoubleSupplier triggerValue) 
  {
    m_triggerValue = triggerValue;
    addRequirements(RobotContainer.noteDetectionPhtnVisionSubsystem);
  }

  @Override
  public void initialize() {
        phtn = RobotContainer.noteDetectionPhtnVisionSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_triggerValue.getAsDouble() > .1){
      phtn.getPHTNData();
      //System.out.println("::::::::::NOTE YAW: "+phtn.getNoteYaw());
      //System.out.println("::::::::::ROBOT YAW: "+RobotContainer.imuSubsystem.getYaw());
      //System.out.println("::::::IS NOTE VISIBLE?: "+phtn.isNoteVisible());
      if (phtn.isNoteVisible()){
        // UNCOMMENT TO TEST ROTATING TO THE ANGLE
        //new TurnToDegreeIMU( phtn.getNoteYaw(), RobotContainer.driveSubsystem, false);
        //System.out.println("::::::::::NOTE YAW: "+phtn.getNoteYaw());
        double visibleYaw = phtn.getNoteYaw()-60;
        if ((visibleYaw>-1)&&(visibleYaw<1)){
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 1);
          recentDetection = true;

        } else if ((visibleYaw > -8)&&(visibleYaw< -1)){
          // TOO FAR LEFT, TURN ON RIGHT RUMBLE
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, .3);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          recentDetection = true;

        } else if ((visibleYaw < 8)&&(visibleYaw > 1)){
          // TOO FAR RIGHT, TURN ON LEFT RUMBLE
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, .3);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          recentDetection = true;

        } else {
          // TOO FAR OFF - TURN OFF ALL RUMBLE
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          recentDetection = false;

         } 
      } else {
        // no note is visible so turn off rumble
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
        RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
        RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
        recentDetection = false;

      }
    } else {
      // no note is visible so turn off rumble

      if (recentDetection) {
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
        RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
        RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
        recentDetection = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}