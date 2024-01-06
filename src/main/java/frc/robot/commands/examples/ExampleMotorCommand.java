// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.examples;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleMotorCommand extends Command {

  public final ExampleSubsystem exampleSubsystem;
  private DoubleSupplier joystickY;

  //this command runs a motor
  public ExampleMotorCommand(ExampleSubsystem exampleSubsystem,  DoubleSupplier joystickY) {
   this.exampleSubsystem = exampleSubsystem;
   this.joystickY = joystickY;
   addRequirements(exampleSubsystem);
  }

 
  @Override
  public void initialize() {
  }

 
  @Override
  public void execute() {
    //Output will depend on the y-axis of joystick
    exampleSubsystem.setCustomPower(joystickY.getAsDouble());
  }


  @Override
  public void end(boolean interrupted) {
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
