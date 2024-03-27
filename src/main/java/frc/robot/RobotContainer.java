package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.commands.auto.BlueNoteEight;
import frc.robot.commands.auto.BlueAmpSideFourNote;
import frc.robot.commands.auto.BlueFiveNote;
import frc.robot.commands.auto.BlueFourNote;
import frc.robot.commands.auto.BlueShootTaxi;
import frc.robot.commands.auto.RedFiveNote;
import frc.robot.commands.auto.FollowChoreoTrajectory;
import frc.robot.commands.auto.RedAmpSideFourNote;
import frc.robot.commands.auto.RedFourNote;
import frc.robot.commands.auto.RedShootTaxi;
import frc.robot.commands.autodrive.AutoAlignWithAmp;
import frc.robot.commands.characterization.WheelRadiusCharacterization;
import frc.robot.commands.characterization.WheelRadiusCharacterization.Direction;
import frc.robot.commands.auto.RedNoteEight;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.commands.shooter.ManualPivot;
import frc.robot.commands.shooter.ManualRollers;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.commands.shooter.ShootWhileMove;
import frc.robot.commands.shooter.SpinUpForSpeaker;
import frc.robot.commands.shooter.SubwooferShot;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final Joystick driverJoystick = new Joystick(JoystickConstants.DRIVER_JOYSTICK_ID);
  private final Joystick operatorJoystick = new Joystick(JoystickConstants.OPERATOR_JOYSTICK_ID);
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final LEDSubsystem ledSubsystem;

  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    SmarterDashboardRegistry.initialize();
    visionSubsystem = new VisionSubsystem();
    driveSubsystem = new DriveSubsystem(); 
    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    pivotSubsystem = new PivotSubsystem();
    ledSubsystem = new LEDSubsystem();

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("red 4note", new RedFourNote(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue 4note", new BlueFourNote(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("red shoot+taxi", new RedShootTaxi(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue shoot+taxi", new BlueShootTaxi(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("fendershot", new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false, ledSubsystem));
    autoChooser.addOption("Red far note 8", new RedNoteEight(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue far note 8", new BlueNoteEight(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue amp side 4 note", new BlueAmpSideFourNote(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, ledSubsystem));
    autoChooser.addOption("red amp side 4 note", new RedAmpSideFourNote(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, ledSubsystem));
    autoChooser.addOption("simple fwdback", new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "simple fwdback", true));
    autoChooser.addOption("1mtrrot", new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "1mtrrot", true));
    autoChooser.addOption("red five note", new RedFiveNote(driveSubsystem, visionSubsystem, shooterSubsystem,intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue five note", new BlueFiveNote(driveSubsystem, visionSubsystem, shooterSubsystem,intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("nothing", null);

    SmartDashboard.putData("autoChooser", autoChooser);
    
    ledSubsystem.setProcess(LEDProcess.DEFAULT);

    DataLogManager.start();
  }
  
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxisCubed(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, HardwareConstants.DEADBAND_VALUE);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double[] modifyAxisCubedPolar(DoubleSupplier xJoystick, DoubleSupplier yJoystick) {
    double xInput = deadband(xJoystick.getAsDouble(), HardwareConstants.DEADBAND_VALUE);
    double yInput = deadband(yJoystick.getAsDouble(), HardwareConstants.DEADBAND_VALUE);
    if (Math.abs(xInput) > 0 && Math.abs(yInput) > 0) {
      double theta = Math.atan(xInput / yInput);
      double hypotenuse = Math.sqrt(xInput * xInput + yInput * yInput);
      double cubedHypotenuse = Math.pow(hypotenuse, 3);
      xInput = Math.copySign(Math.sin(theta) * cubedHypotenuse, xInput);
      yInput = Math.copySign(Math.cos(theta) * cubedHypotenuse, yInput);
      return new double[]{xInput, yInput};
    }
    return new double[]{ Math.copySign(xInput * xInput * xInput, xInput),  Math.copySign(yInput * yInput * yInput, yInput)};
  }

  public void teleopInit() {
    configureButtonBindings();
    SmarterDashboardRegistry.initialize();
  }

  private void configureButtonBindings() {
    // drive
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_X_ID);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_Y_ID);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.RIGHT_STICK_X_ID);
    DoubleSupplier driverLeftStick[] = new DoubleSupplier[]{()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[0], ()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[1]};
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, JoystickConstants.RIGHT_BUMPER_ID);
    POVButton driverRightDirectionPad = new POVButton(driverJoystick, JoystickConstants.RIGHT_D_PAD_ID);
    POVButton driverLeftDirectionPad = new POVButton(driverJoystick, 270);

    // autodrive
    JoystickButton driverAButton = new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID);

    // intake
    Trigger operatorLeftTrigger = new Trigger(() -> (operatorJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    JoystickButton operatorLeftBumper = new JoystickButton(operatorJoystick, JoystickConstants.LEFT_BUMPER_ID);

    // amp and speaker
    JoystickButton operatorBButton = new JoystickButton(operatorJoystick, JoystickConstants.B_BUTTON_ID);
    JoystickButton operatorRightBumper = new JoystickButton(operatorJoystick, JoystickConstants.RIGHT_BUMPER_ID);
    Trigger operatorRightTrigger = new Trigger(() -> (operatorJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));
    Trigger driverRightTrigger = new Trigger(() -> (driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));


    // manual pivot and intake rollers 
    JoystickButton operatorAButton = new JoystickButton(operatorJoystick, JoystickConstants.A_BUTTON_ID);
    JoystickButton operatorXButton = new JoystickButton(operatorJoystick, JoystickConstants.X_BUTTON_ID);
    JoystickButton operatorYButton = new JoystickButton(operatorJoystick, JoystickConstants.Y_BUTTON_ID);
    DoubleSupplier operatorRightStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.RIGHT_STICK_Y_ID);

    // unused
    POVButton operatorUpDirectionPad = new POVButton(operatorJoystick, 0);
    POVButton operatorLeftDirectionPad = new POVButton(operatorJoystick, 270);
    POVButton operatorDownDirectionPad = new POVButton(operatorJoystick, 180);
    Trigger driverLeftTrigger = new Trigger(() -> (driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    JoystickButton driverLeftBumper = new JoystickButton(driverJoystick, 5);
    JoystickButton driverBButton = new JoystickButton(driverJoystick, JoystickConstants.B_BUTTON_ID);
    JoystickButton driverYButton = new JoystickButton(driverJoystick, JoystickConstants.Y_BUTTON_ID);
    DoubleSupplier operatorLeftStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.LEFT_STICK_Y_ID);



    //DRIVER BUTTONS

    // driving
    Command driveCommand = new Drive(driveSubsystem, visionSubsystem,
      driverLeftStick[1],
      driverLeftStick[0],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);

    driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem));
    // Amp Lineup
    driverAButton.whileTrue(new AutoAlignWithAmp(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, driverLeftStick));
    // Spinup for shoot
    // driverRightTrigger.whileTrue(new SpinUpForSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    
    driverLeftBumper.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    driverRightTrigger.whileTrue(new ShootWhileMove(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStick, driverYButton, ledSubsystem));

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverRightDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 
          Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())))));
    // Reset robot odometry based on vision pose measurement from april tags
    driverLeftDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(visionSubsystem.getPoseFromAprilTags())));

    // OPERATOR BUTTONS

    // speaker
    operatorRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    // amp
    operatorRightBumper.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem, ledSubsystem, operatorBButton));
    // fender shot
    operatorUpDirectionPad.whileTrue(new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightStickX, driverRightBumper, ledSubsystem));
    // intake (aka SUCC_BUTTON)
    operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem));
    // outtake (aka UNSUCC_BUTTON)
    operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, true, ledSubsystem));
    // manual pivot (possible climb, unlikely)
    operatorAButton.whileTrue(new ManualPivot(pivotSubsystem, ()->modifyAxisCubed(operatorRightStickY)));
    // manual rollers
    operatorYButton.whileTrue(new ManualRollers(intakeSubsystem, true));
    operatorXButton.whileTrue(new ManualRollers(intakeSubsystem, false));
  }

  public Command getAutonomousCommand() {
    SmarterDashboardRegistry.initialize();
    // Resets the pose factoring in the robot side
    // This is just a failsafe, pose should be reset at the beginning of auto
    driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 
      Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())));
    return autoChooser.getSelected();
  }
}