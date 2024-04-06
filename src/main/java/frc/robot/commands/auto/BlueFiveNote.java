package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.SubwooferShot;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueFiveNote extends SequentialCommandGroup {
  /** Creates a new FiveNotePath. */
  public BlueFiveNote(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d(1.3384751922607414, 5.519354820251465, Rotation2d.fromRadians(0)))),
      new SpinUpForSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, null, null, null, ledSubsystem).withTimeout(0.01),  
      new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to shoot pos", false),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.3),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue center to note 3", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(2.5)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.4),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue 3 to shoot", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(1.3)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.5),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue shoot to note 2", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(2.3)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(1.4),
      new ParallelCommandGroup(
        new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue note 2 to shoot", false),
        new IntakeAuto(intakeSubsystem, pivotSubsystem, shooterSubsystem, ledSubsystem).withTimeout(4.4)
      ),
      new ShootSpeakerAuto(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ledSubsystem).withTimeout(3),
      new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem)
    );
  }
}
