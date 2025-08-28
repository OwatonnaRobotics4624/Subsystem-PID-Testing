// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ScoringMecanisms.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * ZeroElevator command.
 */
public class ZeroElevator extends Command {

    private ElevatorSubsystem m_elevator;
    private final DigitalInput input = new DigitalInput(0);
    /**
     * ZeroElevator command constructor.
     */
    public ZeroElevator(ElevatorSubsystem elevator) {
        m_elevator = elevator;

        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setElevatorOpenLoopDutyCycle(ElevatorCalibrations.kResetPositionVelocity);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_elevator.setZeroPosition();
        }
        m_elevator.setElevatorZeroDutyCycle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !input.get();
    }
}