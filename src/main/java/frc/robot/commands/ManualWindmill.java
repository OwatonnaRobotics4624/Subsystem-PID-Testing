// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.subsystems.ScoringMecanisms.ElevatorSubsystem;
import frc.robot.subsystems.ScoringMecanisms.WindmillSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * ZeroElevator command.
 */
public class ManualWindmill extends Command {

    private WindmillSubsystem m_windmill;
    private double m_speed;
    /**
     * ZeroElevator command constructor.
     */
    public ManualWindmill(WindmillSubsystem windmill, double speed) {
        m_windmill = windmill;
        m_speed = speed;
        addRequirements(m_windmill);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_speed >-1 && m_speed < 1) {
            m_windmill.manualMove(m_speed);
        } else {
            m_windmill.manualMove(0);
        }
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}