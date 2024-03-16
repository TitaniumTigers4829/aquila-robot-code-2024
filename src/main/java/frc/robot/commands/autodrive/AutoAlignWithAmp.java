package frc.robot.commands.autodrive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoAlignWithAmp extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private boolean isRed;
  private Pose2d ampPose;

  private final ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );

  private final ProfiledPIDController translationController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
  );

  /** Creates a new AutoAlignWithAmp. */
  public AutoAlignWithAmp(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // This will default to blue if it alliance isn't present
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    ampPose = isRed ? new Pose2d(FieldConstants.RED_AMP_SHOOT_X, FieldConstants.RED_AMP_SHOOT_Y, FieldConstants.RED_AMP_ROTATION) 
      : new Pose2d(FieldConstants.BLUE_AMP_SHOOT_X, FieldConstants.BLUE_AMP_SHOOT_Y, FieldConstants.BLUE_AMP_ROTATION);
  }

  @Override
  public void execute() {
    super.execute();

    // Gets the error between the desired pos (the amp) and the current pos of the robot
    Pose2d drivePose = driveSubsystem.getPose();
    double xPoseError = ampPose.getX() - drivePose.getX();
    double yPoseError = ampPose.getY() - drivePose.getY();
    double thetaPoseError = ampPose.getRotation().getRadians() - drivePose.getRotation().getRadians();

    // Uses the PID controllers to calculate the drive output
    double xOutput = deadband(translationController.calculate(xPoseError, 0));
    double yOutput = deadband(translationController.calculate(yPoseError, 0));
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    double turnOutput = deadband(turnController.calculate(thetaPoseError, 0)); 

    // Drives the robot towards the amp
    driveSubsystem.drive(
      xOutput,
      yOutput,
      turnOutput, 
      true
    );
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  private double deadband(double val) {
    if (Math.abs(val) < 0.1) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
