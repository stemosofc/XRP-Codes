// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BangBang extends Command {
  private final XRPDrivetrain drivetrain;
  private final double distance;
  private double error;
  /** Creates a new BangBang. */
  public BangBang(XRPDrivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;

    SmartDashboard.putNumber("Target", distance);
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualDistance = drivetrain.getLeftDistanceInch();
    error = distance - actualDistance;

    if(error > 0) {
      drivetrain.arcadeDrive(1, 0);
    } else if (error < 0) {
      drivetrain.arcadeDrive(1, 0);
    }

    SmartDashboard.putNumber("Actual Position", actualDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (error == 0);
  }
}
