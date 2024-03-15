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

public class ShootWhileMove extends DriveCommandBase {
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;

  private final DoubleSupplier[] leftStick;
  private final BooleanSupplier isFieldRelative;

  private double headingError = 0;

  private final ProfiledPIDController turnController = new ProfiledPIDController(
    ShooterConstants.AUTO_SHOOT_P,
    ShooterConstants.AUTO_SHOOT_I, 
    ShooterConstants.AUTO_SHOOT_D, 
    ShooterConstants.AUTO_SHOOT_CONSTRAINTS
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
    // if alliance is detected
    if (alliance.isPresent()) {
      // and if it's red, we're red
      isRed = alliance.get() == Alliance.Red;
    } else {
      // otherwise default to red alliance
      isRed = true;
    }
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
    double dt = robotPos3d.getDistance(speakerPos) / ShooterConstants.NOTE_LAUNCH_VELOCITY;
    // For shooting while moving, we can pretend that our robot is stationary, but has traveled
    // the distance that was how long the note was in the air for times the robots current velocity
    double dx = speeds.vxMetersPerSecond * dt;
    double dy = speeds.vyMetersPerSecond * dt;
    // get the heading of the robot so we can do some fun trig    
    double theta = driveSubsystem.getOdometryRotation2d().getRadians();
    // dx, dy are robot relative, rotated counterclockwise from horizontal Θ radians
    // adjust the robot pos using:
    // x = x + ((dx * cos(Θ)) - (dy * sin(Θ)))
    // y = y + ((dy * cos(Θ)) - (dx * sin(Θ)))
    robotPose2d = robotPose2d.plus(new Translation2d((dx * Math.cos(theta)) - (dy * Math.sin(theta)), (dy * Math.cos(theta)) + (dx * Math.sin(theta))));
    // continue the command as normal
    double distance = robotPose2d.getDistance(speakerPos.toTranslation2d());
    desiredHeading = Math.atan2(robotPos3d.getX() - speakerPos.getX(), robotPos3d.getY() - speakerPos.getY());
    // heading error
    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();
    
    // deadband output for a steadier shot when we are really close to the target
    double turnOutput = deadband(turnController.calculate(headingError, 0));

    // allow the driver to drive at full speed (fancy math go brrr (hopefully))
    // reason for using a DoubleSupplier[] here is so that the modifyAxisCubedPolar can be used
    driveSubsystem.drive(
      leftStick[0].getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
      leftStick[1].getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND, 
      turnOutput, 
      !isFieldRelative.getAsBoolean()
    );

    // spin up the shooter
    shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    // set the pivot
    pivotSubsystem.setPivotFromDistance(distance);
    // if we are ready to shoot:
    if (isReadyToShoot()) {
      leds.setProcess(LEDProcess.SHOOT);
      shooterSubsystem.setTowerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setTowerSpeed(0);
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError() && (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_RADIANS);
  }

  private double deadband(double val) {
    if (Math.abs(val) < 0.05) {
      return 0.0;
    } else {
      return val;
    }
  } 
}
