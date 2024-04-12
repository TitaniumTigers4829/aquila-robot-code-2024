package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueFarSideFourNote extends SequentialCommandGroup {
  /** Creates a new FiveNotePath. */
  public BlueFarSideFourNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(0.7656821853637688, 4.516969680786133, Rotation2d.fromRadians(-0.9913894645352261)))),
      new SpinUpForSpeakerAuto(shooterSubsystem, pivotSubsystem, null).withTimeout(0.01),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.0),
      new ParallelCommandGroup(
            new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redfar3note 1", false),
            new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(5.6)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.3),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redfar3note 2", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(5.7)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.4),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "redfar3note 3", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(4.6)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem)
    );
  }
}
