package frc.robot;

import java.util.function.DoubleSupplier;

import javax.xml.crypto.Data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
// import frc.robot.commands.auto.BlueNoteEight;
import frc.robot.commands.auto.BlueAmpSideFourNote;
import frc.robot.commands.auto.BlueFiveNote;
import frc.robot.commands.auto.BlueFourNote;
import frc.robot.commands.auto.BlueShootTaxi;
import frc.robot.commands.auto.BlueUnderStage4Note;
import frc.robot.commands.auto.RedFiveNote;
import frc.robot.commands.auto.RedAmpSideFourNote;
// import frc.robot.commands.auto.RedFar3Note;
import frc.robot.commands.auto.RedFarSideFourNote;
import frc.robot.commands.auto.RedFourNote;
import frc.robot.commands.auto.RedShootTaxi;
import frc.robot.commands.auto.RedSixNote;
// import frc.robot.commands.auto.RedUnderStage4note;
import frc.robot.commands.auto.RedUnderStage4Note;
import frc.robot.commands.auto.StopShooterAndIntake;
import frc.robot.commands.autodrive.AutoAlignWithAmp;
import frc.robot.commands.auto.RedNoteEight;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.ManualIntake;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.extras.characterization.WheelRadiusCharacterization;
import frc.robot.extras.characterization.WheelRadiusCharacterization.Direction;
import frc.robot.commands.shooter.ManualPivot;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootPass;
import frc.robot.commands.shooter.ShootSpeaker;
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
  private final XboxController driverController = new XboxController(JoystickConstants.DRIVER_JOYSTICK_ID);
  private final XboxController operatorController = new XboxController(JoystickConstants.OPERATOR_JOYSTICK_ID);
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
    autoChooser.addOption("red shoot+taxi", new RedShootTaxi(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue shoot+taxi", new BlueShootTaxi(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("fendershot", new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, ()->0, ()->0, ()->0, ()->false, ledSubsystem));
    autoChooser.addOption("Red far note 8", new RedNoteEight(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    // autoChooser.addOption("blue far note 8", new BlueNoteEight(driveSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue amp side 4 note", new BlueAmpSideFourNote(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, ledSubsystem));
    autoChooser.addOption("red amp side 4 note", new RedAmpSideFourNote(driveSubsystem, visionSubsystem, shooterSubsystem, pivotSubsystem, intakeSubsystem, ledSubsystem));
    // autoChooser.addOption("simple fwdback", new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "simple fwdback", true));
    // autoChooser.addOption("1mtrrot", new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "1mtrrot", true));
    autoChooser.addOption("red five note", new RedFiveNote(driveSubsystem, visionSubsystem, shooterSubsystem,intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("blue five note", new BlueFiveNote(driveSubsystem, visionSubsystem, shooterSubsystem,intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("red under stage 4 note", new RedUnderStage4Note(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, intakeSubsystem, ledSubsystem));
    autoChooser.addOption("blue under stage 4 note", new BlueUnderStage4Note(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, intakeSubsystem, ledSubsystem));
    // autoChooser.addOption("red far 3 note", new RedFar3Note(driveSubsystem, visionSubsystem, pivotSubsystem, shooterSubsystem, intakeSubsystem));
    autoChooser.addOption("red six", new RedSixNote(driveSubsystem, visionSubsystem, shooterSubsystem, intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("red far side four", new RedFarSideFourNote(driveSubsystem, visionSubsystem, shooterSubsystem, intakeSubsystem, pivotSubsystem, ledSubsystem));
    autoChooser.addOption("nothing", null);

    SmartDashboard.putData("autoChooser", autoChooser);
    
    ledSubsystem.setProcess(LEDProcess.DEFAULT);

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
    shooterSubsystem.setFlywheelNeutral();
    shooterSubsystem.setRollerSpeed(0);
    intakeSubsystem.setFlapperSpeed(0);
    intakeSubsystem.setIntakeSpeed(0);
    pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
    visionSubsystem.setTeleopStatus(true);
    // new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem).schedule();
  }

  public void intakeCallback(boolean hasNote) {
    if (hasNote) {
      driverController.setRumble(RumbleType.kBothRumble, 0.1);
      operatorController.setRumble(RumbleType.kBothRumble, 1);
    } else {
      driverController.setRumble(RumbleType.kBothRumble, 0);
      operatorController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  private void configureButtonBindings() {
    DoubleSupplier driverLeftStickX = driverController::getLeftX;
    DoubleSupplier driverLeftStickY = driverController::getLeftY;
    DoubleSupplier driverRightStickX = driverController::getRightX;
    DoubleSupplier driverLeftStick[] = new DoubleSupplier[]{()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[0], ()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[1]};

    Trigger driverRightBumper = new Trigger(driverController::getRightBumper);
    Trigger driverRightDirectionPad = new Trigger(()->driverController.getPOV() == 90);
    Trigger driverLeftDirectionPad = new Trigger(()->driverController.getPOV() == 270);
    // JoystickButton driverRightBumper = new JoystickButton(driverController, JoystickConstants.RIGHT_BUMPER_ID);
    // POVButton driverRightDirectionPad = new POVButton(driverController, JoystickConstants.RIGHT_D_PAD_ID);
    // POVButton driverLeftDirectionPad = new POVButton(driverController, 270);

    // autodrive
    Trigger driverAButton = new Trigger(driverController::getAButton);
    // JoystickButton driverAButton = new JoystickButton(driverController, JoystickConstants.A_BUTTON_ID);

    // intake
    Trigger operatorLeftTrigger = new Trigger(()->operatorController.getLeftTriggerAxis() > 0.2);
    Trigger operatorLeftBumper = new Trigger(operatorController::getLeftBumper);
    // Trigger operatorLeftTrigger = new Trigger(() -> (operatorController.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    // JoystickButton operatorLeftBumper = new JoystickButton(operatorController, JoystickConstants.LEFT_BUMPER_ID);

    // amp and speaker
    Trigger operatorBButton = new Trigger(operatorController::getBButton);
    Trigger operatorRightBumper = new Trigger(operatorController::getRightBumper);
    Trigger operatorRightTrigger = new Trigger(()->operatorController.getRightTriggerAxis() > 0.2);
    Trigger driverRightTrigger = new Trigger(()->driverController.getRightTriggerAxis() > 0.2);
    // JoystickButton operatorBButton = new JoystickButton(operatorController, JoystickConstants.B_BUTTON_ID);
    // JoystickButton operatorRightBumper = new JoystickButton(operatorController, JoystickConstants.RIGHT_BUMPER_ID);
    // Trigger operatorRightTrigger = new Trigger(() -> (operatorController.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));
    // Trigger driverRightTrigger = new Trigger(() -> (driverController.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));


    // manual pivot and intake rollers 
    Trigger operatorAButton = new Trigger(operatorController::getAButton);
    Trigger operatorXButton = new Trigger(operatorController::getXButton);
    Trigger operatorYButton = new Trigger(operatorController::getYButton);
    DoubleSupplier operatorRightStickY = operatorController::getRightY;
    // JoystickButton operatorAButton = new JoystickButton(operatorController, JoystickConstants.A_BUTTON_ID);
    // JoystickButton operatorXButton = new JoystickButton(operatorController, JoystickConstants.X_BUTTON_ID);
    // JoystickButton operatorYButton = new JoystickButton(operatorController, JoystickConstants.Y_BUTTON_ID);
    // DoubleSupplier operatorRightStickY = () -> operatorController.getRawAxis(JoystickConstants.RIGHT_STICK_Y_ID);

    // unused
    Trigger operatorUpDirectionPad = new Trigger(()->operatorController.getPOV() == 0);
    Trigger operatorLeftDirectionPad = new Trigger(()->operatorController.getPOV() == 270);
    Trigger operatorDownDirectionPad = new Trigger(()->operatorController.getPOV() == 180);
    Trigger driverLeftTrigger = new Trigger(()->driverController.getLeftTriggerAxis() > 0.2);
    Trigger driverLeftBumper = new Trigger(driverController::getLeftBumper);
    Trigger driverBButton = new Trigger(driverController::getBButton);
    Trigger driverYButton = new Trigger(driverController::getYButton);
    DoubleSupplier operatorLeftStickY = operatorController::getLeftY;
    // POVButton operatorUpDirectionPad = new POVButton(operatorController, 0);
    // POVButton operatorLeftDirectionPad = new POVButton(operatorController, 270);
    // POVButton operatorDownDirectionPad = new POVButton(operatorController, 180);
    // Trigger driverLeftTrigger = new Trigger(() -> (driverController.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    // JoystickButton driverLeftBumper = new JoystickButton(driverController, 5);
    // JoystickButton driverBButton = new JoystickButton(driverController, JoystickConstants.B_BUTTON_ID);//used for wheelcharacterisation
    // JoystickButton driverYButton = new JoystickButton(driverController, JoystickConstants.Y_BUTTON_ID);//used for intakefromshooter
    // DoubleSupplier operatorLeftStickY = () -> operatorController.getRawAxis(JoystickConstants.LEFT_STICK_Y_ID);



    //DRIVER BUTTONS

    // driving

    Command driveCommand = new Drive(driveSubsystem, visionSubsystem,
      driverLeftStick[1],
      driverLeftStick[0],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean(),
      () -> driverLeftBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);
    // shooterSubsystem.setDefaultCommand(new FlywheelSpinUpAuto(shooterSubsystem, visionSubsystem));

    driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // driverLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.3));
    // Amp Lineup
    driverAButton.whileTrue(new AutoAlignWithAmp(driveSubsystem, visionSubsystem, driverLeftStick));
    // Spinup for shoot
    driverRightTrigger.whileTrue(new SpinUpForSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    
    // driverLeftBumper.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // driverRightTrigger.whileTrue(new ShootWhileMove(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStick, driverYButton, ledSubsystem));

    // Resets the robot angle in the odometry, factors in which alliance the robot is on
    driverRightDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 
          Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())))));
    // // Reset robot odometry based on vision pose measurement from april tags
    driverLeftDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(visionSubsystem.getLastSeenPose())));
    // driverLeftDpad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(15.251774787902832, 5.573054313659668, Rotation2d.fromRadians(3.14159265)))));
    // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));

    // driverXButton.
    driverBButton.whileTrue(new WheelRadiusCharacterization(driveSubsystem, Direction.CLOCKWISE));
    // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickY, operatorLeftStickY, driverYButton, ledSubsystem));
    // driverDownDirectionPad.whileTrue(new IntakeFromShooter(shooterSubsystem, intakeSubsystem));
    driverYButton.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // OPERATOR BUTTONS

    // speaker
    operatorRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    // amp
    operatorRightBumper.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem, ledSubsystem, operatorBButton));
    // fender shot
    operatorUpDirectionPad.whileTrue(new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightStickX, driverRightBumper, ledSubsystem));
    // intake (aka SUCC_BUTTON)
    operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // operatorLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.2));
    // outtake (aka UNSUCC_BUTTON)
    operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, true, ledSubsystem, this::intakeCallback));
    // manual pivot (possible climb, unlikely)
    operatorAButton.whileTrue(new ManualPivot(pivotSubsystem, ()->modifyAxisCubed(operatorRightStickY)));
    operatorDownDirectionPad.whileTrue(new ManualPivot(pivotSubsystem, ()->-0.25));
    // manual rollers
    operatorYButton.whileTrue(new ManualIntake(intakeSubsystem, true));
    operatorXButton.whileTrue(new ManualIntake(intakeSubsystem, false));

    // operatorBButton.onTrue(new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));
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