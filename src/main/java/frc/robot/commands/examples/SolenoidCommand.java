// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.examples;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ExampleSubsystem;

//This command extends the solenoid... it can also be done inline within the subsystem..
public class SolenoidCommand extends InstantCommand {

  private final ExampleSubsystem exampleSubsystem;

  public SolenoidCommand(ExampleSubsystem exampleSubsystem) {
    this.exampleSubsystem = exampleSubsystem;
    addRequirements(exampleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    exampleSubsystem.extendSolenoid();
  }


}
