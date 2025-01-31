// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeadReckoning extends Command {
  /** Creates a new DeadReckoing. */

  private final XRPDrivetrain drivetrain;
  private final double maxTime;
  private final double distance;

  private Timer timer = new Timer();

  public DeadReckoning(XRPDrivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;
    maxTime = distance / drivetrain.getMaxVelocityInch();

    addRequirements(drivetrain);
    drivetrain.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target", distance);
    SmartDashboard.putBoolean("Finished", false);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(1.0, 0);

    SmartDashboard.putNumber("Actual Position", drivetrain.getLeftDistanceInch());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Finished", true);
    drivetrain.arcadeDrive(0, 0);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= maxTime;
  }
}
