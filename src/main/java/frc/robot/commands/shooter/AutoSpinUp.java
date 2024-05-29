// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoSpinUp extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final LEDSubsystem leds;

  private final DoubleSupplier leftX, leftY, rightX;
  private final BooleanSupplier isFieldRelative;

  private double headingError = 0;

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation2d speakerPos;

  /** Creates a new SpinUpForSpeaker. */
  public AutoSpinUp(
      DriveSubsystem driveSubsystem,
      ShooterSubsystem shooterSubsystem,
      PivotSubsystem pivotSubsystem,
      VisionSubsystem visionSubsystem,
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      BooleanSupplier isFieldRelative,
      LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    this.isFieldRelative = isFieldRelative;
    this.leds = leds;
    addRequirements(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // if alliance is detected
    if (alliance.isPresent()) {
      // and if it's red, we're red
      isRed = alliance.get() == Alliance.Red;
    } else {
      // otherwise default to red alliance
      isRed = true;
    }
    // SmartDashboard.putBoolean("red", isRed);
    speakerPos =
        isRed
            ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y)
            : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    // SmartDashboard.putString("speakerPos", speakerPos.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

    // get positions of various things
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // distance (for speaker lookups)
    double distance = robotPos.getDistance(speakerPos);
    // arctangent for desired heading
    desiredHeading =
        Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));

    // current
    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();

    // allow the driver to drive slowly (NOT full speed - will mess up shooter)
    driveSubsystem.drive(
        deadband(leftY.getAsDouble()) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        deadband(leftX.getAsDouble()) * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
        deadband(rightX.getAsDouble()) * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        !isFieldRelative.getAsBoolean());

    // if we are ready to shoot:
    if (isInRange()) {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM * .75);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isInRange() {
    if (desiredHeading > 25) {
      return true;
    }
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
