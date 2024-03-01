// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.auto.FollowChoreoTrajectory;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.commands.shooter.ManualPivot;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.commands.shooter.TestShooter;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
  
  public RobotContainer() {
    SmarterDashboardRegistry.initialize();
    visionSubsystem = new VisionSubsystem();
    driveSubsystem = new DriveSubsystem(); 
    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    pivotSubsystem = new PivotSubsystem();
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
    value = deadband(value, 0.1);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double[] modifyAxisCubedPolar(DoubleSupplier xJoystick, DoubleSupplier yJoystick) {
    double xInput = deadband(xJoystick.getAsDouble(), 0.2);
    double yInput = deadband(yJoystick.getAsDouble(), 0.2);
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
    // driveSubsystem.zeroHeading();
    // driveSubsystem.resetOdometry(driveSubsystem.getPose());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // drive
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_X_ID);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.LEFT_STICK_Y_ID);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.RIGHT_STICK_X_ID);
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, JoystickConstants.RIGHT_BUMPER_ID);
    POVButton driverRightDirectionPad = new POVButton(driverJoystick, JoystickConstants.RIGHT_D_PAD_ID);
    JoystickButton driverLeftBumper = new JoystickButton(driverJoystick, 5);

    // autodrive
    JoystickButton driverAButton = new JoystickButton(driverJoystick, JoystickConstants.A_BUTTON_ID);
    JoystickButton driverBButton = new JoystickButton(driverJoystick, JoystickConstants.B_BUTTON_ID);

    JoystickButton driverYButton = new JoystickButton(driverJoystick, JoystickConstants.Y_BUTTON_ID);

    // shoot
    Trigger driverLeftTrigger = new Trigger(() -> (driverJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    JoystickButton operatorAButton = new JoystickButton(operatorJoystick, JoystickConstants.A_BUTTON_ID);
    DoubleSupplier operatorLeftStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.LEFT_STICK_Y_ID);
    // amp
    JoystickButton operatorYButton = new JoystickButton(operatorJoystick, JoystickConstants.Y_BUTTON_ID);

    // intake
    Trigger driverRightTrigger = new Trigger(() -> (driverJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));
    Trigger operatorRightTrigger = new Trigger(() -> (operatorJoystick.getRawAxis(JoystickConstants.RIGHT_TRIGGER_ID) > 0.2));
    Trigger operatorLeftTrigger = new Trigger(() -> (operatorJoystick.getRawAxis(JoystickConstants.LEFT_TRIGGER_ID) > 0.2));
    JoystickButton operatorBButton = new JoystickButton(operatorJoystick, JoystickConstants.B_BUTTON_ID);
    JoystickButton operatorLeftBumper = new JoystickButton(operatorJoystick, JoystickConstants.LEFT_BUMPER_ID);
    JoystickButton operatorRightBumper = new JoystickButton(operatorJoystick, JoystickConstants.RIGHT_BUMPER_ID);
    // autoclimb
    JoystickButton operatorXButton = new JoystickButton(operatorJoystick, JoystickConstants.X_BUTTON_ID);
    // manualclimb
    DoubleSupplier operatorRightStickY = () -> operatorJoystick.getRawAxis(JoystickConstants.RIGHT_STICK_Y_ID);

    POVButton operatorDpadUp = new POVButton(operatorJoystick, 0);
    POVButton operatorDpadDown = new POVButton(operatorJoystick, 180);

    // manual driving
    Command driveCommand = new Drive(driveSubsystem, visionSubsystem,
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[0],
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[1],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);
    
    driverRightDirectionPad.onTrue(new InstantCommand(driveSubsystem::zeroHeading));
    driverRightDirectionPad.onTrue(new InstantCommand(()->driveSubsystem.resetOdometry(driveSubsystem.getPose())));

    // // realtime trajectories
    // // amp
    // driverAButton.whileTrue(new DriveToAmp(driveSubsystem, visionSubsystem, driverAButton));
    // // speaker
    // driverBButton.whileTrue(new DriveToPos(driveSubsystem, visionSubsystem, driverBButton, new Pose2d(SmarterDashboardRegistry.getSpeakerPos(), driveSubsystem.getRotation2d())));
    // // loading station
    // driverBButton.whileTrue(new DriveToPos(driveSubsystem, visionSubsystem, driverBButton, new Pose2d(SmarterDashboardRegistry.getLoadingStationPos(), driveSubsystem.getRotation2d())));

    // shoot
    driverRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper));
    operatorRightBumper.whileTrue(new TestShooter(shooterSubsystem));

    // operatorRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper));

    // intake
   
    driverLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false));
    // operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false));
    driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, true));
    // operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false));

    // Shoot amp
    // driverAButton.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem));

    // climb
    // operatorXButton.onTrue(new InstantCommand(() -> pivotSubsystem.setPivot(PivotConstants.PIVOT_START_CLIMB_ANGLE)));
    // operatorXButton.onFalse(new InstantCommand(() -> pivotSubsystem.setPivot(PivotConstants.PIVOT_END_CLIMB_ANGLE)));

    operatorAButton.whileTrue(new ManualPivot(pivotSubsystem, ()->modifyAxisCubed(operatorRightStickY)));
  }

  public Command getAutonomousCommand() {
    // return null;
    return new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "blue to note 1", true);
  }
}