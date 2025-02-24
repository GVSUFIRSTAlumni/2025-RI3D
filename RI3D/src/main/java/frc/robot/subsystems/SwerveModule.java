package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SubsystemBase {
    private final SparkMax m_driveMotor, m_steerMotor;

    private final RelativeEncoder m_driveEncoder, m_steerEncoder;

    private final SparkClosedLoopController m_drivePID;
    private final SparkClosedLoopController m_steerPID;
    private final SimpleMotorFeedforward m_driveFF;

    private final CANcoder m_CANCoder;

    private double m_encoderOffset;

    private SwerveModuleState m_curState;

    public final int modNum;

    /**
     * 
     * @param driveID       CAN ID of the drive motor
     * @param steerID       CAN ID of the steer motor
     * @param encoderID     CAN ID of the CANCoder
     * @param encoderOffset starting encoder position
     */
    public SwerveModule(final int driveID, final int steerID, final int encoderID, final double encoderOffset, int swerveID) {
        m_driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        m_steerMotor = new SparkMax(steerID, MotorType.kBrushless);       
        m_CANCoder = new CANcoder(encoderID);
        m_encoderOffset = encoderOffset;
        System.out.println(swerveID + " " + steerID);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_steerEncoder = m_steerMotor.getEncoder();
        
        m_driveFF = new SimpleMotorFeedforward(0.667, 2.44, 0.27);

        // TODO see what this changed to
        // m_steerEncoder.setPositionConversionFactor((1/12.8) * 2 * Math.PI);
        // m_driveEncoder.setVelocityConversionFactor(((Units.inchesToMeters(4) * Math.PI) / 6.75) / 60);

        m_drivePID = m_driveMotor.getClosedLoopController();
        m_steerPID = m_steerMotor.getClosedLoopController();
        // m_drivePID = new PIDController(0.1, 0, 0);
        // m_steerPID = new PIDController(.5, 0, 0); // TODO (old): set these
        // m_steerPID.enableContinuousInput(-Math.PI, Math.PI);
        
        // m_steerPID.setTolerance(0.0004); //10 degree tolerance


        modNum = swerveID;

        SmartDashboard.putNumber("encoder offset " + modNum, m_encoderOffset);

        configSteer();
        resetEncoders();

        configDrive();
    }

    /**
     * 
     * @param state 
     */
    public void setDesiredState(SwerveModuleState state)  {
        setSpeed(state);
        setAngle(state);
    }

    /**
     * Gets the speed of the drive motor.
     * @return the current speed in meters per second.
     */
    private double getSpeed() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * 
     * @param state
     */
    public void setSpeed(SwerveModuleState state) {
        m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, m_driveFF.calculate(state.speedMetersPerSecond));
        // m_drivePID.setSetpoint(state.speedMetersPerSecond);
    }

    /**
     * 
     * @return 
     */
    public double getAngleRelative() {
        // 360 degrees in a circle divided by 4096 encoder counts/revolution (CANCoder resolution)
        // return (m_CANCoder.getAbsolutePosition() * 360 / 4096) - m_encoderOffset.getDegrees();
        return m_steerEncoder.getPosition();
    }

    public double getAngleAbsolute() {
        return (m_CANCoder.getAbsolutePosition().getValueAsDouble() - m_encoderOffset) * 2 * Math.PI;
    }

    /**
     * 
     * @param state
     */
    private void setAngle(SwerveModuleState state) {
        //Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainConstants.maxSpeed * 0.01)) ? m_lastAngle : state.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = state.angle;
        // m_steerPID.setSetpoint(angle.getRadians());
        m_steerPID.setReference(angle.getRadians(), ControlType.kPosition);
    }

    private void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_steerEncoder.setPosition(getAngleAbsolute());
        m_steerPID.setReference(0, ControlType.kPosition);
        m_drivePID.setReference(0.5, ControlType.kVelocity, ClosedLoopSlot.kSlot0, m_driveFF.calculate(0.5));
    }

    /**
     * 
     */
    private void configDrive() {
        var config = new SparkMaxConfig();

        config.idleMode(DrivetrainConstants.DriveParams.kIdleMode);
        config.encoder
            .positionConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 6.75)))
            .velocityConversionFactor((((Units.inchesToMeters(4) * Math.PI) / 6.75) / 60)); // in meters per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DrivetrainConstants.DriveParams.kP,
                DrivetrainConstants.DriveParams.kI,
                DrivetrainConstants.DriveParams.kD,
                DrivetrainConstants.DriveParams.kFF
            );
        config.smartCurrentLimit(60, 30);
        //0 is front left swerve
        //2 is back left swerve
        if (modNum == 0 || modNum == 2) {
            config.inverted(true);
            System.out.println("SwerveID config checked");
        }

        m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void configSteer() {
        var config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor((1/12.8) * 2 * Math.PI);
        
        config.closedLoop
            .pid(0.5,0,0);
        
        m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // make the CANCoder return a value (0,1) instead of (-0.5,0.5)
        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        m_CANCoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.withAbsoluteSensorDiscontinuityPoint(1);
        m_CANCoder.getConfigurator().apply(encoderConfig);
    }

    public void ReZero(){
        m_steerEncoder.setPosition(getAngleAbsolute());   
    }

    @Override
    public void periodic() {
        // m_encoderOffset = SmartDashboard.getNumber("encoder offset " + modNum, 0);
        // SmartDashboard.putNumber("relative encoder " + modNum + " ", m_driveEncoder.getPosition());
        // resetEncoders();
    }
}
