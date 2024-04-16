package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Drive extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier leftJoystickY, leftJoystickX, rightJoystickX;
  private final BooleanSupplier isFieldRelative, isHighRotation;
  private double angularSpeed;

  private SlewRateLimiter forwardLimiter = new SlewRateLimiter(10.0);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(10.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(10.0);
  
  /**
   * The command for driving the robot using joystick inputs.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive
   * field relative
   */
  public Drive(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier leftJoystickY, DoubleSupplier leftJoystickX, DoubleSupplier rightJoystickX, BooleanSupplier isFieldRelative, BooleanSupplier isHighRotation) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
    this.isHighRotation = isHighRotation;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Drives the robot
    if (isHighRotation.getAsBoolean()) {
      angularSpeed = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
    } else {
      angularSpeed = DriveConstants.LOW_ANGULAR_SPEED_RADIANS_PER_SECOND;
    }
    
    driveSubsystem.drive(
      forwardLimiter.calculate(leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND),
      rightLimiter.calculate(leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND),
      rotationLimiter.calculate(rightJoystickX.getAsDouble() * angularSpeed),
      isFieldRelative.getAsBoolean()
    );

    // Runs all the code from DriveCommandBase that estimates pose
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    // When the command ends, it stops the robot
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
