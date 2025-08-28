// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Calibrations.ElevatorCalibrations;
import frc.robot.Calibrations.WindmillCalibrations;
import frc.robot.subsystems.ScoringMecanisms.ElevatorSubsystem;
import frc.robot.subsystems.ScoringMecanisms.WindmillSubsystem;

/**
 * L4 command.
 */
public class L2 extends SequentialCommandGroup {

    /**
     * CoralStation command constructor.
     */
    public L2(ElevatorSubsystem elevator, WindmillSubsystem windmill) {
        super(
            new MoveElevatorToPosition(ElevatorCalibrations.kL2Position, ElevatorCalibrations.kL2Tolerance, elevator),
            new MoveWindmillToPosition(WindmillCalibrations.kL2Position, ElevatorCalibrations.kL2Tolerance, windmill)
        );
    }

}