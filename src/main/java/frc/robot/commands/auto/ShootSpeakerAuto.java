package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.drive.DriveCommandBase;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

public class ShootSpeakerAuto extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;
  private final Timer pewPewCompleteTimer;

  private final Timer pivotTimer;

  private double headingError = 0;

  private final ProfiledPIDController turnController =
      new ProfiledPIDController(
          ShooterConstants.AUTO_SHOOT_P,
          ShooterConstants.AUTO_SHOOT_I,
          ShooterConstants.AUTO_SHOOT_D,
          ShooterConstants.AUTO_SHOOT_CONSTRAINTS);

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;

  /** Creates a new ShootSpeaker. */
  public ShootSpeakerAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leds = leds;
    pewPewCompleteTimer = new Timer();
    pivotTimer = new Timer();
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pewPewCompleteTimer.stop();
    pewPewCompleteTimer.reset();
    pivotTimer.stop();
    pivotTimer.reset();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // sets alliance to red
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    speakerPos =
        isRed
            ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y)
            : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    // intakeSubsystem.setIntakeSpeed(-0.2);

    // get positions of various things
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    desiredHeading = Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));

    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();

    turnController.enableContinuousInput(-Math.PI, Math.PI);
    double turnOutput = deadband(turnController.calculate(headingError, 0));

    driveSubsystem.drive(
      0,
      0,
      turnOutput,
      false
    );

    if (distance > 3.2) {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_FAR_RPM);
    } else if (distance > 1.8) {
      shooterSubsystem.setRPM(4400);
    } else {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    }

    pivotSubsystem.setPivotFromSpeakerDistance(distance);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      leds.setProcess(LEDProcess.SHOOT);
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }

    // If it has shot the note and the timer hasn't started
    if (!shooterSubsystem.hasNote() && !pewPewCompleteTimer.hasElapsed(0.001)) {
      pewPewCompleteTimer.start();
    }

    if (pivotSubsystem.isPivotWithinAcceptableError() && !pivotTimer.hasElapsed(0.001)) {
      pivotTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM * 0.5);
    shooterSubsystem.setRollerSpeed(0);
    // intakeSubsystem.setIntakeSpeed(0);
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pewPewCompleteTimer.hasElapsed(0.1);
    // return !shooterSubsystem.hasNote();
  }

  public boolean isReadyToShoot() {
    return shooterSubsystem.isShooterWithinAcceptableError() && pivotTimer.hasElapsed(0.1) && Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_RADIANS;
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  }
}
