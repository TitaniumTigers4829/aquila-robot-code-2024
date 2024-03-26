// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class ShootWhileMove extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;

  private final DoubleSupplier[] leftStick;
  private final BooleanSupplier isFieldRelative;

  private double headingError = 0;

  private final ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_MOVE_P,
    ShooterConstants.AUTO_SHOOT_MOVE_I, 
    ShooterConstants.AUTO_SHOOT_MOVE_D, 
    ShooterConstants.AUTO_SHOOT_MOVE_CONSTRAINTS
  );

  private boolean isRed = false;
  private double desiredHeading = 0;
  private Translation3d speakerPos;
  
  /** Creates a new ShootSpeaker. */
  public ShootWhileMove(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier[] leftStick, BooleanSupplier isFieldRelative, LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leftStick = leftStick;
    this.isFieldRelative = isFieldRelative;
    this.leds = leds;
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem, visionSubsystem);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Sets isRed to true if alliance is red
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;  

    speakerPos = isRed ? new Translation3d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y, ShooterConstants.SPEAKER_HEIGHT) 
      : new Translation3d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y, ShooterConstants.SPEAKER_HEIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update odometry and useful things like that
    super.execute();
    
      // TODO: someone check my logic here
    // current pose
    Translation2d robotPose2d = driveSubsystem.getPose().getTranslation();
    Translation3d robotPos3d = new Translation3d(robotPose2d.getX(), robotPose2d.getY(), ShooterConstants.SHOOTER_HEIGHT);
    // speeds
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    // this gets the time that the note will be in the air between the robot and the speaker
    // x = v * t
    // t = x / v
    double tmpDist = robotPos3d.getDistance(speakerPos);
    double dt = tmpDist / ShooterConstants.NOTE_LAUNCH_VELOCITY;
    // For shooting while moving, we can pretend that our robot is stationary, but has traveled
    // the distance that was how long the note was in the air for times the robots current velocity
    double dx = speeds.vxMetersPerSecond * dt * (tmpDist * 1.1);
    double dy = speeds.vyMetersPerSecond * dt * (tmpDist * 1.1);
    // account for the current velocity:
    robotPose2d.plus(new Translation2d(dx, dy).rotateBy(driveSubsystem.getOdometryRotation2d()));
    // continue the command as normal
    double distance = robotPose2d.getDistance(speakerPos.toTranslation2d());
    desiredHeading = Math.atan2(robotPose2d.getY() - speakerPos.getY(), robotPose2d.getX() - speakerPos.getX());
    // heading error
    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();
    
    // deadband output for a steadier shot when we are really close to the target
    double turnOutput = deadband(turnController.calculate(headingError, 0));

    // allow the driver to drive at full speed (fancy math go brrr (hopefully))
    // reason for using a DoubleSupplier[] here is so that the modifyAxisCubedPolar can be used
    driveSubsystem.drive(
      leftStick[1].getAsDouble() * DriveConstants.MAX_SHOOT_SPEED_METERS_PER_SECOND, 
      leftStick[0].getAsDouble() * DriveConstants.MAX_SHOOT_SPEED_METERS_PER_SECOND, 
      turnOutput, 
      !isFieldRelative.getAsBoolean()
    );

    // spin up the shooter
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    pivotSubsystem.setPivotFromDistance(distance);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      leds.setProcess(LEDProcess.SHOOT);
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
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
  public boolean isReadyToShoot() {
    SmartDashboard.putBoolean("shooter", shooterSubsystem.isShooterWithinAcceptableError());
    SmartDashboard.putBoolean("pivot", pivotSubsystem.isPivotWithinAcceptableError());
    SmartDashboard.putBoolean("heading", Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_MOVING_RADIANS);
    return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError() && (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_MOVING_RADIANS);
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
