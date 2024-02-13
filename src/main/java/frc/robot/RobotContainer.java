// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.TowerIntake;
import frc.robot.commands.shooter.RollerSpeedSetter;
import frc.robot.commands.shooter.RunShooterPower;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

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

    DoubleSupplier operatorLeftStickX = () -> operatorJoystick.getRawAxis(0);
    DoubleSupplier operatorRightStickX = () -> operatorJoystick.getRawAxis(4);
    JoystickButton xOperatorButton = new JoystickButton(operatorJoystick, 3);
        
    JoystickButton yDriverButton = new JoystickButton(driverJoystick, 4);
    JoystickButton aDriverButton = new JoystickButton(driverJoystick, 1);
    
    JoystickButton bDriverButton = new JoystickButton(driverJoystick, 2);
    
    JoystickButton yOperatorButton = new JoystickButton(operatorJoystick, 4);
    JoystickButton aOperatorButton = new JoystickButton(operatorJoystick, 1);

    Command driveCommand = new Drive(driveSubsystem, visionSubsystem,
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[0],
      () -> modifyAxisCubedPolar(driverLeftStickY, driverLeftStickX)[1],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean()
    );

    Command shoot = new RunShooterPower(shooterSubsystem,
      () -> modifyAxisCubedPolar(operatorLeftStickX, operatorRightStickX)[0]
    );

    driverRightDpad.onTrue(new InstantCommand(() ->driveSubsystem.zeroHeading()));
    driverRightDpad.onTrue(new InstantCommand(()->driveSubsystem.resetOdometry(new Pose2d())));


    bDriverButton.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));

    driveSubsystem.setDefaultCommand(driveCommand);

    aOperatorButton.whileTrue(new RollerSpeedSetter(shooterSubsystem));
    yOperatorButton.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));

    shooterSubsystem.setDefaultCommand(shoot);

    yDriverButton.onTrue(new InstantCommand(() -> pivotSubsystem.set(0.05)));
    aDriverButton.onTrue(new InstantCommand(() -> pivotSubsystem.set(-0.05))); 
    aDriverButton.onFalse(new InstantCommand(() -> pivotSubsystem.set(0)));
    yDriverButton.onFalse(new InstantCommand(() -> pivotSubsystem.set(0)));

    xOperatorButton.onTrue(new FeedForwardCharacterization(driveSubsystem, driveSubsystem::setCharacterizationVoltage, driveSubsystem::getCharacterizationVelocity));
    
  }



  public Command getAutonomousCommand() {
    return null;
}
}