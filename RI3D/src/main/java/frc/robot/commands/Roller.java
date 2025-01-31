// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Roller extends Command {
    AlgaeRoller m_roller;

    /** Creates a new Roller. */
    public Roller(AlgaeRoller roller) {
        m_roller = roller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_roller);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_roller.setVelocity(0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_roller.setVelocity(0.5);
        // System.out.println("THEY'RE TAKING THE HOBBITS TO ISENGUARD!!");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_roller.setVelocity(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
