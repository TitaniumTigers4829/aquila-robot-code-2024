package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.NoteDetector;

//Gets ROBOT RELATIVE POSE (NOT FIELD RELATIVE POSE) of Note
public class GetNotePos extends Command {

  public GetNotePos() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d testPose = NoteDetector.findNotePose();
    SmartDashboard.putString("note pos", testPose.toString());
    double[] testArray = NoteDetector.getWorldPts();
    SmartDashboard.putString("x and y", Arrays.toString(testArray));

    Pose2d notePose =  NoteDetector.findNotePose();
    Translation2d cameraOffset = NoteDetector.applyCameraOffset(notePose);
    SmartDashboard.putString("note pos", cameraOffset.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}