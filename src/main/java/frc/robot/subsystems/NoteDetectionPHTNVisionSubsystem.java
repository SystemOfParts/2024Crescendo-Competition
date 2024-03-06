// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Iterator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.commands.XboxRumbleCommand;
import frc.robot.lib.VisionHelpers;

public class NoteDetectionPHTNVisionSubsystem extends SubsystemBase implements VisionHelpers {

  // private noteFieldLayout noteFieldLayout =
  // noteFields.k2024Crescendo.loadnoteLayoutField(); // load 2024 field

  private String cameraName;
  private PhotonCamera camera;
  private PhotonPipelineResult noteResult;
  private boolean noteHasTargets;
  private List<PhotonTrackedTarget> noteTargets;
  private PhotonTrackedTarget noteBestTarget;
  // private PhotonPoseEstimator poseEstimator;
  private Transform3d robotToCam;
  private double noteX, noteY, noteYaw;
  private static double noteZAngle;
  private static double noteZ = -1;
  private Pose2d globalPoseEstimate = new Pose2d();
  private Transform3d fieldToCamera;

  /** Creates a new PhotonVisionSubsystem. */
  
  public NoteDetectionPHTNVisionSubsystem(String cameraName) {

    // Check if we have PV installed
    if (!PhotonVisionConstants.NOTE_PV_PRESENT) {
        System.out.println("----=======> Note PV Not Working");
      return;
    }

    this.cameraName = cameraName;
    camera = new PhotonCamera("NoteCamera");
    noteResult = new PhotonPipelineResult();
    noteHasTargets = false;
    System.out.println("----=======> PV Working");
  }
  public Pose2d getRobotFieldPosePV() {

    noteResult = camera.getLatestResult();
    noteHasTargets = noteResult.hasTargets();

    if (noteHasTargets) {
      noteTargets = noteResult.getTargets();
      noteBestTarget = noteResult.getBestTarget();

      noteX = noteBestTarget.getBestCameraToTarget().getX();
      noteY = noteBestTarget.getBestCameraToTarget().getY();
      noteZ = noteBestTarget.getBestCameraToTarget().getZ();
      noteZAngle = noteBestTarget.getBestCameraToTarget().getRotation().getAngle();
      fieldToCamera = noteResult.getMultiTagResult().estimatedPose.best;

      if (noteResult.getMultiTagResult().estimatedPose.isPresent) { // this may need to be commented out as it depends whether the single tag pose estimation is enabled

        globalPoseEstimate = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(),
          new Rotation2d(fieldToCamera.getRotation().getX(), fieldToCamera.getRotation().getY()));
          // apriltaField2d.setRobotPose(globalPoseEstimate);
        return globalPoseEstimate;
      } 

    } 
    return null; // if no notes visible or the pose cannot be determined
  }

   public void getPHTNData() {
    //System.out.println("GetPHTNDATA");
    noteResult = camera.getLatestResult();
    noteHasTargets = noteResult.hasTargets();
    //System.out.println("----=======> PHTN DATA hasTargets: "+noteHasTargets);
    if (noteHasTargets) {
      //System.out.println("----=======> PHTN DATA TARGETS WERE FOUND: ");
      noteTargets = noteResult.getTargets();
      noteBestTarget = noteResult.getBestTarget();
      noteX = noteBestTarget.getBestCameraToTarget().getX();
      noteY = noteBestTarget.getBestCameraToTarget().getY();
      noteZ = noteBestTarget.getBestCameraToTarget().getZ();
      noteYaw = noteBestTarget.getYaw();
      noteZAngle = noteBestTarget.getBestCameraToTarget().getRotation().getAngle();
      //fieldToCamera = noteResult.getMultiTagResult().estimatedPose.best;
    }
  }

  public boolean isNoteVisible() {
    return this.noteHasTargets;
  }

  /**
   * Gets the X coordinate of the note in meters.
   * 
   * @return The X coordinate.
   */
  public double getNoteX() {
    return noteX;
  }

  /**
   * Gets the Y coordinate of the note in meters.
   * 
   * @return The Y coordinate.
   */
  public double getNoteY() {
    return noteY;
  }

  /**
   * Gets the Z coordinate of the note in meters.
   * 
   * @return The Z coordinate.
   */
  public  double getNoteZ() {
    return noteZ;
  }

  /**
   * Gets the Z angle of the note in degrees.
   * 
   * @return The Z angle.
   */
  public double getNoteZAngle() {

    return noteZAngle*(180/Math.PI);

  }

  public double getNoteYaw() {

    return noteYaw;

  }


  @Override
  public void periodic() {
    /* if (RobotContainer.isRightTriggerPressed()){
      getPHTNData();

      //System.out.println("]]]]]]]]----=======> IS A NOTE VISIBLE? : "+(isnoteVisible()));
      if (isnoteVisible()){
        double visibleYaw = getNoteYaw();
        //System.out.println("]]]]]]]]]]]]----=======> Note WAS VISIBLE ITS YAW: ["+getNoteYaw()+"]");
        if ((visibleYaw > -3) && (visibleYaw < 3)){
          //System.out.println("----=======> [ GOOD NOTE WITHIN 3 DEGREES ] =======------");
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 1);
        } else if ((visibleYaw > -8)){
          // TOO FAR LEFT, TURN ON RIGHT RUMBLE
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, .3);
        } else if ((visibleYaw < 8)){
          // TOO FAR RIGHT, TURN ON LEFT RUMBLE
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, .3);
        } else {
          // TOO FAR OFF - TURN OFF ALL RUMBLE
          //System.out.println("----=======> [ GOOD NOTE BUT YAW IS TOO FAR OFF ] =======------");
          RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
          RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
        }
      } else {
        // no note is visible so turn off rumble
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
        RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
        RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
      }
    } else {
        // TRIGGER RELEASED, so turn off rumble
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0); 
        RobotContainer.xboxController.setRumble(RumbleType.kLeftRumble, 0);
        RobotContainer.xboxController.setRumble(RumbleType.kRightRumble, 0);
    }*/
  } 
}