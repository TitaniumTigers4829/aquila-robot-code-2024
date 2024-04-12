// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.PivotConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.pivot.PivotSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem;

// public class ShootTrap extends Command {
//   private ShooterSubsystem shooterSubsystem;
//   private final PivotSubsystem pivotSubsystem;
//   /** Creates a new ShootTrap. */
//   public ShootTrap(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem) {
//     this.shooterSubsystem = shooterSubsystem;
//     this.pivotSubsystem = pivotSubsystem;
//     addRequirements(shooterSubsystem, pivotSubsystem);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     shooterSubsystem.runBlower(1);
//     shooterSubsystem.setRPM(3000);
//     pivotSubsystem.setPivotAngle(PivotConstants.SHOOT_TRAP_ANGLE);
//     if (isReadyToShoot()) {
//       shooterSubsystem.setRollerSpeed(ShooterConstants.ROLLER_SHOOT_SPEED);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     shooterSubsystem.runBlower(0);
//     shooterSubsystem.setRPM(0);
//     pivotSubsystem.setPivotAngle(PivotConstants.PIVOT_INTAKE_ANGLE);
//     shooterSubsystem.setRollerSpeed(0);
//   }

//   public boolean isReadyToShoot() {
//      return shooterSubsystem.isShooterWithinAcceptableError() && pivotSubsystem.isPivotWithinAcceptableError();
//   }


//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
