// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MotionProfile extends Command {

  private final XRPDrivetrain drivetrain;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
  private final TrapezoidProfile.Constraints constraintsTrapezoid = new TrapezoidProfile.Constraints(0.3048, 0.200);
  private final ProfiledPIDController motionProfile = new ProfiledPIDController(2, 0, 0, constraintsTrapezoid);

  /** Creates a new MotionProfile. */
  public MotionProfile(XRPDrivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    motionProfile.setGoal(Units.inchesToMeters(distance));
    SmartDashboard.putNumber("Target", distance);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = drivetrain.getLeftDistanceInch();
    double output = motionProfile.calculate(Units.inchesToMeters(distance)) + feedforward.calculate(motionProfile.getSetpoint().velocity);
    drivetrain.arcadeDrive(output, 0);

    SmartDashboard.putNumber("Actual Position", distance);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motionProfile.atGoal();
  }
}
