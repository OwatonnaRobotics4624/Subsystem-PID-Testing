// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringMecanisms.WindmillSubsystem;

/**
 * MoveWindmillToPosition Command.
 */
public class MoveWindmillToPosition extends Command {
    private WindmillSubsystem m_windmill;

    private double m_newSetpoint;
    private double m_tolerance;

    /**
     * MoveWindmillToPosition command constructor.
     */
    public MoveWindmillToPosition(double newSetpoint, double tolerance, WindmillSubsystem windmill) {
        m_newSetpoint = newSetpoint;
        m_tolerance = tolerance;
        m_windmill = windmill;

        addRequirements(m_windmill);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_windmill.updateSetpoint(m_newSetpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //System.out.println("Checking MoveWindmillToPosition");
        return m_windmill.isWithinTolerance(m_tolerance);
    }
}