package frc.robot.commands.shooter;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

public class shooterPassing extends DriveCommandBase {

//initiates the variables
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem leds;

  private final DoubleSupplier leftX, leftY;
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
  
  public ShootSpeaker(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, BooleanSupplier isFieldRelative, LEDSubsystem leds) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.leftX = leftX;
    this.leftY = leftY;
    this.isFieldRelative = isFieldRelative;
    this.leds = leds;
    addRequirements(shooterSubsystem, driveSubsystem, pivotSubsystem, visionSubsystem);
  }
  
@Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    //sets alliance to red
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;  

    speakerPos = isRed ? new Translation2d(FieldConstants.RED_PASSING_X, FieldConstants.RED_PASSING_Y) : new Translation2d(FieldConstants.BLUE_PASSING_X, FieldConstants.BLUE_PASSING_Y);
  }

  @Override
  public void execute() {
    super.execute();
    
    //positions + distance
    Translation2d robotPos = driveSubsystem.getPose().getTranslation();
    double distance = robotPos.getDistance(speakerPos);
    
    desiredHeading = Math.atan2((robotPos.getY() - speakerPos.getY()), (robotPos.getX() - speakerPos.getX()));
    headingError = desiredHeading - driveSubsystem.getOdometryRotation2d().getRadians();
  
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    double turnOutput = deadband(turnController.calculate(headingError, 0));
    
    driveSubsystem.drive(
      deadband(leftY.getAsDouble()) * 0.5, deadband(leftX.getAsDouble()) * 0.5, turnOutput,!isFieldRelative.getAsBoolean()
    );
    
    if (distance > 4.2) {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_FAR_RPM);
    } else {
      shooterSubsystem.setRPM(ShooterConstants.SHOOT_SPEAKER_RPM);
    }
    
    pivotSubsystem.setPivotFromPassDistance(distance);
    if (isReadyToShoot()) {
      leds.setProcess(LEDProcess.SHOOT);
      shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
    } else {
      leds.setProcess(LEDProcess.FINISH_LINE_UP);
    }
  }

  @Override
  public void end(boolean interrupted) {
    leds.setProcess(LEDProcess.DEFAULT);
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    leds.setProcess(LEDProcess.DEFAULT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  public boolean isReadyToShoot() {
    // TODO: heading
     return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError() && (Math.abs(headingError) < DriveConstants.HEADING_ACCEPTABLE_ERROR_RADIANS);
  }

  private double deadband(double val) {
    if (Math.abs(val) < HardwareConstants.DEADBAND_VALUE) {
      return 0.0;
    } else {
      return val;
    }
  } 
  
  }
