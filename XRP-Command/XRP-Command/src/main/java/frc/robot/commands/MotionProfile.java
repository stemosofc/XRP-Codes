// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MotionProfile extends Command {

  private final XRPDrivetrain drivetrain;
  private final SimpleMotorFeedforward feedforward;
  private final TrapezoidProfile.Constraints constraintsTrapezoid;
  private final ProfiledPIDController motionProfile;
  private final double distance;
  /** Creates a new MotionProfile. */
  public MotionProfile(XRPDrivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;

    addRequirements(drivetrain);

    feedforward = new SimpleMotorFeedforward(0.08, 0.084, 0.01);
    constraintsTrapezoid = new TrapezoidProfile.Constraints(drivetrain.getMaxVelocityInch(), 12);
    motionProfile = new ProfiledPIDController(2, 0, 0, constraintsTrapezoid);

    motionProfile.setGoal(distance);
    motionProfile.setTolerance(0.7);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target", distance);
    SmartDashboard.putBoolean("Finished", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = drivetrain.getLeftDistanceInch();
    double desiredVelocity = motionProfile.getSetpoint().velocity;
    double output = motionProfile.calculate(distance) + feedforward.calculate(desiredVelocity);
    drivetrain.setVoltage(output);

    SmartDashboard.putNumber("Actual Position", distance);
    SmartDashboard.putNumber("Desired velocity", desiredVelocity);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Finished", true);
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motionProfile.atGoal();
  }
}
