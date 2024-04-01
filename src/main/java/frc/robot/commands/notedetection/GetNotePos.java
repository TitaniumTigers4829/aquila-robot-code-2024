package frc.robot.commands.notedetection;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.NoteDetector;


//Gets ROBOT RELATIVE POSE (NOT FIELD RELATIVE POSE) of Note
public class GetNotePos extends Command {
  /** Creates a new GetNotePos. */
  private final NoteDetector noteDetector;

  public GetNotePos(NoteDetector noteDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.noteDetector = noteDetector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose2d testPose = noteDetector.findNotePose();
    // SmartDashboard.putString("note pos", testPose.toString());
    double[] testArray = noteDetector.getWorldPts();
    SmartDashboard.putString("x and y", Arrays.toString(testArray));

    Translation2d notePos = noteDetector.applyCameraOffset(noteDetector.findNotePose());
    SmartDashboard.putString("note pos", notePos.toString());

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