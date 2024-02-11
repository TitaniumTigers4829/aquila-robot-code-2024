// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.autodrive.DriveToPos;
import frc.robot.commands.autodrive.NewDriveToPos;
import frc.robot.commands.autodrive.NewSquaredDriveToPos;
import frc.robot.commands.autonomous.FollowChoreoTrajectory;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.TestThings;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.commands.shooter.RollerSpeedSetter;
import frc.robot.commands.shooter.RunShooterPower;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  /*
   * TODOs
   * Finish fancy auto
   *   verify followpathandshoot.java
   *   make new paths
   *   figure out max shoot distance
   *   make shootspeakerauto.java
   *   make intakeauto.java
   *   add the follow path commands
   * 
   * make simpler autos (raina and juno)
   */

  private final VisionSubsystem visionSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  
  public RobotContainer() {
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
    double xInput = deadband(xJoystick.getAsDouble(), 0.1);
    double yInput = deadband(yJoystick.getAsDouble(), 0.1);
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
  }

  private void configureButtonBindings() {

    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_X);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_LEFT_STICK_Y);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(JoystickConstants.DRIVER_RIGHT_STICK_X);
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, JoystickConstants.DRIVER_RIGHT_BUMPER_ID);
    POVButton driverRightDpad = new POVButton(driverJoystick, 90);

    DoubleSupplier operatorStickX = () -> operatorJoystick.getRawAxis(0);

    Command driveCommand = new Drive(driveSubsystem, visionSubsystem,
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[0],
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[1],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean()
    );

    JoystickButton bJoystickButton = new JoystickButton(driverJoystick, 2);

    bJoystickButton.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));

    driveSubsystem.setDefaultCommand(driveCommand);

    JoystickButton yButton = new JoystickButton(operatorJoystick, 4);
    JoystickButton aButton = new JoystickButton(operatorJoystick, 1);
    aButton.whileTrue(new RollerSpeedSetter(shooterSubsystem));
    yButton.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));

    // JoystickButton yOperatorButton = new JoystickButton(operatorJoystick, 4);
    shooterSubsystem.setDefaultCommand(new RunShooterPower(shooterSubsystem, operatorStickX));

    JoystickButton yDriverButton = new JoystickButton(driverJoystick, 4);
    JoystickButton aDriverButton = new JoystickButton(driverJoystick, 1);

    yDriverButton.onTrue(new InstantCommand(() -> pivotSubsystem.set(0.1)));
    aDriverButton.onTrue(new InstantCommand(() -> pivotSubsystem.set(-0.1))); 
    aDriverButton.onFalse(new InstantCommand(() -> pivotSubsystem.set(0)));
    yDriverButton.onFalse(new InstantCommand(() -> pivotSubsystem.set(0)));


  }

  public Command getAutonomousCommand() {
    return new FollowChoreoTrajectory(driveSubsystem, visionSubsystem, "rotate");
  }
}