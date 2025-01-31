package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.ElevatorParams;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// TODO
// - set up the PID loop
// - set up encoder conversions

public class Elevator extends SubsystemBase {
    private final SparkMax m_elevator;
    private final RelativeEncoder m_internalEncoder;
    private final RelativeEncoder m_externalEncoder;
    private final ProfiledPIDController m_controller;

    private int messageNum = 0;
    private boolean overrideFlag = false;
    private double outputSpeed = 0;
    
    public Elevator() {
        // TODO: Tune trapezoid profile
        m_controller = new ProfiledPIDController(ElevatorParams.kP, ElevatorParams.kI, ElevatorParams.kD, new TrapezoidProfile.Constraints(60 * ElevatorParams.elevatorSpeed, 50 * ElevatorParams.elevatorSpeed));
        
        m_elevator = new SparkMax(ElevatorParams.elevatorMotorID, MotorType.kBrushless);
        m_internalEncoder = m_elevator.getEncoder();
        m_externalEncoder = m_elevator.getAlternateEncoder();

        configMotor();

        m_internalEncoder.setPosition(0);
        m_externalEncoder.setPosition(0);
        
        m_controller.reset(0);
    }

    /**
     * Set the position of the elevator
     * @param pos the position in inches
     */
    public void setPositionGoal(double pos) {
        // TODO: Remove this check as redundance with enableContinuousInput?
        if (pos > ElevatorParams.maxHeight)
        {
            System.out.printf("cannot set position to height %f - max height is %f!\n", pos, ElevatorParams.maxHeight);
        }
        else
        {
            m_controller.setGoal(pos);
        }
    }

    public void overrideUp()
    {
        outputSpeed = ElevatorParams.elevatorSpeed;
        overrideFlag = true;
    }

    public void overrideDown()
    {
        outputSpeed = -ElevatorParams.elevatorSpeed;
        overrideFlag = true;
    }

    public void stop()
    {
        outputSpeed = 0;
        overrideFlag = false;
    }

    public double getPosition()
    {
        return m_externalEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if(!overrideFlag){
            outputSpeed = m_controller.calculate(m_externalEncoder.getPosition());
        }

        if (messageNum >= 50)
        {
            // System.out.printf("Attempting to output to elevator: %f\n", outputSpeed);
            // System.out.printf("Alternate Elevator Encoder: %.4f\n", m_externalEncoder.getPosition());
            messageNum = 0;
        }
        else
        {
            ++messageNum;
        }

        m_elevator.set(outputSpeed);
    }

    private void configMotor() {
        var config = new SparkMaxConfig();

        config.idleMode(ElevatorParams.kIdleMode);
        config.encoder // TODO: Setup the desired units we are using <---- DO NOT FORGET UNITS
            .positionConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 25)))
            .velocityConversionFactor((((Units.inchesToMeters(1.9) * Math.PI) / 25) / 60)); // in meters per second
        config.alternateEncoder
            .positionConversionFactor((((Units.inchesToMeters(0.5) * Math.PI)))) // TODO: Remove the 16
            .velocityConversionFactor((((Units.inchesToMeters(0.5) * Math.PI)) / 60)); // in meters per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pidf(
                ElevatorParams.kP,
                ElevatorParams.kI,
                ElevatorParams.kD,
                ElevatorParams.kFF
            );
        // config.smartCurrentLimit(60, 30);
        config.inverted(true); // True is the correct way

        m_elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}

