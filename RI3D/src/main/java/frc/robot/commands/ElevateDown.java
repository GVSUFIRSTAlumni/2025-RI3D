package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ElevateDown extends InstantCommand{
    private final Elevator m_elevator;
    public ElevateDown(Elevator elevator) {

         m_elevator = elevator;
         addRequirements(m_elevator);
    }
    
    @Override
    public void initialize() {

        m_elevator.setGoal(0); 
        m_elevator.enable(); 
    }

    
}