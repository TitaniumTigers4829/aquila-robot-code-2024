package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class Drive extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier leftJoystickY, leftJoystickX, rightJoystickX;
  private final BooleanSupplier isFieldRelative;

  private SlewRateLimiter yLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter xLimiter = new SlewRateLimiter(10);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(10);


  /**
   * The command for driving the robot using joystick inputs.
   *
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive field relative
   */
  public Drive(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      DoubleSupplier leftJoystickY,
      DoubleSupplier leftJoystickX,
      DoubleSupplier rightJoystickX,
      BooleanSupplier isFieldRelative) {
    super(driveSubsystem, visionSubsystem);
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);
    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Drives the robot
    driveSubsystem.drive(
        yLimiter.calculate(leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND),
        xLimiter.calculate(leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND),
        rotLimiter.calculate(rightJoystickX.getAsDouble() * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND),
        isFieldRelative.getAsBoolean());

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
