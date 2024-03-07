// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
  private Translation2d speakerPos;
  private Timer timer;
  // default to -0.02 because that is the defualt loop time
  private double lastTime = -0.02;
  
  /** Creates a new ShootSpeaker. */
  public ShootWhileMove(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier[] leftStick, BooleanSupplier isFieldRelative, LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leftStick = leftStick;
    this.isFieldRelative = isFieldRelative;
    this.leds = leds;
    timer = new Timer();
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem, visionSubsystem);
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
    speakerPos = isRed ? new Translation2d(FieldConstants.RED_SPEAKER_X, FieldConstants.RED_SPEAKER_Y) : new Translation2d(FieldConstants.BLUE_SPEAKER_X, FieldConstants.BLUE_SPEAKER_Y);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update odometry and useful things like that
    super.execute();
    
    // TODO: someone check my logic here
    // current pose
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    // speeds
    ChassisSpeeds speeds = driveSubsystem.getRobotRelativeSpeeds();
    // how much to adjust the position by
    double dt = timer.get() - lastTime;
    // adjust x and y position
    double dx = speeds.vxMetersPerSecond * dt;
    double dy = speeds.vyMetersPerSecond * dt;
    // get the heading of the robot so we can do some fun trig    
    double theta = driveSubsystem.getOdometryRotation2d().getRadians();
    // adjust the robot pos using:
    // x = x + ((dx * cos(Θ)) - (dy * sin(Θ)))
    // y = y + ((dy * cos(Θ)) - (dx * sin(Θ)))
    robotPos = robotPos.plus(new Translation2d((dx * Math.cos(theta)) - (dy * Math.sin(theta)), (dy * Math.cos(theta)) - (dx * Math.sin(theta))));
    // continue the command as normal
    double distance = robotPos.getDistance(speakerPos);
    // heading error
    headingError = desiredHeading - theta;

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
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }

    lastTime = timer.get();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivot(PivotConstants.PIVOT_INTAKE_ANGLE);
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
