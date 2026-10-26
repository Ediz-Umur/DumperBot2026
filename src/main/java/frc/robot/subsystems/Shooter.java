package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    //The motor, CANCoder & CANRange devices
    private TalonFX m_LeftShooterLeader = new TalonFX(Constants.sShooter.dLeftShooterLeaderID);
    private TalonFX m_LeftFollowerShooter = new TalonFX(Constants.sShooter.dLeftShooterFollowerID);
    private TalonFX m_RightShooterLeader = new TalonFX(Constants.sShooter.dRightShooterLeaderID);
    private TalonFX m_RightFollowerShooter = new TalonFX(Constants.sShooter.dRightShooterFollowerID);
    private TalonFX m_Kicker = new TalonFX(Constants.sShooter.dKickerID);
    private TalonFX m_Hood = new TalonFX(Constants.sShooter.dHoodID);

    private CANcoder m_HoodEncoder = new CANcoder(Constants.sShooter.dHoodEncoder);
    private CANrange m_RangeLeft = new CANrange(Constants.sShooter.dRangeLeft);
    private CANrange m_RangeRight = new CANrange(Constants.sShooter.dRangeRight);
    
    //The configurators for the objects
    private TalonFXConfiguration m_LeftFlywheelConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration m_RightFlywheelConfiguration = new TalonFXConfiguration();
    
    private TalonFXConfiguration m_KickerConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration m_HoodConfiguration = new TalonFXConfiguration();
    private CANcoderConfiguration m_EncoderConfigs = new CANcoderConfiguration();
    private CANrangeConfiguration m_RangeConfiguration = new CANrangeConfiguration();

    private Slot0Configs m_FlywheelSlot0 = m_LeftFlywheelConfiguration.Slot0;
    private Slot0Configs m_KickerSlot0 = m_KickerConfiguration.Slot0;
    private Slot0Configs m_HoodSlot0 = m_HoodConfiguration.Slot0;
    
    //Motion Magic configurators
    private MotionMagicConfigs m_FlywheelMotionConfigs = m_LeftFlywheelConfiguration.MotionMagic;
    private MotionMagicConfigs m_KickerMotionConfigs = m_KickerConfiguration.MotionMagic;
    private MotionMagicConfigs m_HoodMotionConfigs = m_HoodConfiguration.MotionMagic;

    public Shooter(){

        //Motor Configuration
        m_LeftFlywheelConfiguration.Feedback.SensorToMechanismRatio = Constants.sShooter.cFlywheelGearRatio;
        m_HoodConfiguration.Feedback.RotorToSensorRatio = Constants.sShooter.cHoodGearRatioMtE;
        m_HoodConfiguration.Feedback.SensorToMechanismRatio = Constants.sShooter.cHoodGearRatioEtMc;
        m_KickerConfiguration.Feedback.SensorToMechanismRatio = Constants.sShooter.cKickerGearRatio;

        m_KickerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_LeftFlywheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_Hood.setNeutralMode(NeutralModeValue.Brake);

        m_LeftFlywheelConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sShooter.cFlywheelCurrentLimit));
        m_KickerConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sShooter.cKickerCurrentLimit));
        m_HoodConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sShooter.cHoodCurrentLimit));

        //CANcoder Configs (Currently on FUSED, fallback is REMOTE)
        m_EncoderConfigs.MagnetSensor.MagnetOffset = Constants.sShooter.fMagnetOffset;
        m_EncoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.sShooter.fDiscontinuityPoint;
        m_EncoderConfigs.MagnetSensor.SensorDirection = Constants.sShooter.fSensorDirection;
        
        m_HoodConfiguration.Feedback.FeedbackRemoteSensorID = Constants.sShooter.dHoodEncoder;
        m_HoodConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        //CANrange Configuration
        m_RangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = Constants.sShooter.rMinSignalStrength;
        m_RangeConfiguration.ProximityParams.ProximityThreshold = Constants.sShooter.rDetectionRange;

        //PID Gains
        m_FlywheelSlot0.kS = Constants.sShooter.cFlywheelS;
        m_FlywheelSlot0.kA = Constants.sShooter.cFlywheelA;
        m_FlywheelSlot0.kV = Constants.sShooter.cFlywheelV;
        m_FlywheelSlot0.kP = Constants.sShooter.cFlywheelP;
        m_FlywheelSlot0.kI = Constants.sShooter.cFlywheelI;
        m_FlywheelSlot0.kD = Constants.sShooter.cFlywheelD;

        m_KickerSlot0.kS = Constants.sShooter.cKickerkS;
        m_KickerSlot0.kA = Constants.sShooter.cKickerkA;
        m_KickerSlot0.kV = Constants.sShooter.cKickerkV;
        m_KickerSlot0.kP = Constants.sShooter.cKickerkP;
        m_KickerSlot0.kI = Constants.sShooter.cKickerkI;
        m_KickerSlot0.kD = Constants.sShooter.cKickerkD;

        m_HoodSlot0.kG = Constants.sShooter.cHoodG;
        m_HoodSlot0.kS = Constants.sShooter.cHoodS;
        m_HoodSlot0.kA = Constants.sShooter.cHoodA;
        m_HoodSlot0.kV = Constants.sShooter.cHoodV;
        m_HoodSlot0.kP = Constants.sShooter.cHoodP;
        m_HoodSlot0.kI = Constants.sShooter.cHoodI;
        m_HoodSlot0.kD = Constants.sShooter.cHoodD;

        //MotionMagic Gains
        m_FlywheelMotionConfigs.MotionMagicAcceleration = Constants.sShooter.mmFlywheelAcceleration;
        m_FlywheelMotionConfigs.MotionMagicJerk = Constants.sShooter.mmFlywheelJerk;

        m_KickerMotionConfigs.MotionMagicAcceleration = Constants.sShooter.mmKickerAcceleration;
        m_KickerMotionConfigs.MotionMagicJerk = Constants.sShooter.mmKickerJerk;

        m_HoodMotionConfigs.MotionMagicCruiseVelocity = Constants.sShooter.mmHoodCruise;
        m_HoodMotionConfigs.MotionMagicExpo_kA = Constants.sShooter.mmHoodkA;
        m_HoodMotionConfigs.MotionMagicExpo_kV = Constants.sShooter.mmHoodkV;

        //Cloning the identical parts of the left and right flywheel configuratior
        m_RightFlywheelConfiguration.Slot0 = m_LeftFlywheelConfiguration.Slot0;
        m_RightFlywheelConfiguration.MotionMagic = m_LeftFlywheelConfiguration.MotionMagic;
        m_RightFlywheelConfiguration.Feedback = m_LeftFlywheelConfiguration.Feedback;

        m_RightFlywheelConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Applying the configurations
        m_Kicker.getConfigurator().apply(m_KickerConfiguration);
        m_Hood.getConfigurator().apply(m_HoodConfiguration);

        m_LeftShooterLeader.getConfigurator().apply(m_LeftFlywheelConfiguration);
        m_LeftFollowerShooter.getConfigurator().apply(m_LeftFlywheelConfiguration);
        m_RightShooterLeader.getConfigurator().apply(m_RightFlywheelConfiguration);
        m_RightFollowerShooter.getConfigurator().apply(m_RightFlywheelConfiguration);

        m_HoodEncoder.getConfigurator().apply(m_EncoderConfigs);
        m_RangeLeft.getConfigurator().apply(m_RangeConfiguration);
        m_RangeRight.getConfigurator().apply(m_RangeConfiguration);
    }

    //Making the kicker motor move
    public void KickerMove(double Velocity){
        VelocityVoltage m_KickerMotionMagic = new VelocityVoltage(Velocity).withEnableFOC(true);
        m_Kicker.setControl(m_KickerMotionMagic);
    }

    //Making the hood motor move
    public void HoodMove(double Degrees){
        PositionTorqueCurrentFOC m_HoodMotionMagic = new PositionTorqueCurrentFOC(Degrees * Constants.sShooter.kHoodDegreeToRotation);
        m_Hood.setControl(m_HoodMotionMagic);
    }

    /*
    public void FlywheelSysIDMove(double volts){
        m_LeftShooterLeader.setControl(new VoltageOut(volts));
        m_LeftFollowerShooter.setControl(new Follower(Constants.sShooter.dLeftShooterLeaderID, MotorAlignmentValue.Aligned));
        m_RightShooterLeader.setControl(new VoltageOut(volts));
        m_RightFollowerShooter.setControl(new Follower(Constants.sShooter.dRightShooterLeaderID, MotorAlignmentValue.Aligned));
    } */

    public void FlywheelMove(double RPS){
        VelocityVoltage m_FlywheelMotionMagic = new VelocityVoltage(RPS);
        m_LeftShooterLeader.setControl(m_FlywheelMotionMagic);
        m_LeftFollowerShooter.setControl(new Follower(Constants.sShooter.dLeftShooterLeaderID, MotorAlignmentValue.Aligned));
        m_RightShooterLeader.setControl(m_FlywheelMotionMagic);
        m_RightFollowerShooter.setControl(new Follower(Constants.sShooter.dRightShooterLeaderID, MotorAlignmentValue.Aligned));
    }

    //Getting the acceleration of the flywheel
    public double GetFlywheelAcceleration(){
        return(m_LeftShooterLeader.getAcceleration().getValueAsDouble());
    }

    //Getting the velocity of the flywheel
    public double GetFlywheelVelocity(){
        return(m_LeftShooterLeader.getVelocity().getValueAsDouble());
    }

    //Getting the velocity of the kicker
    public double GetKickerVelocity(){
        return(m_Kicker.getVelocity().getValueAsDouble());
    }

    //Getting the angle of the hood
    public double GetHoodAngle(){
        return(m_Hood.getPosition().getValueAsDouble());
    }

    //Gets whether the CANRange detected something
    public boolean BeamBreakDetected(){
        return(m_RangeLeft.getIsDetected().asSupplier().get() && m_RangeRight.getIsDetected().asSupplier().get());
    }

    //Stops kicker motor
    public void KickerStop(){
        m_Kicker.stopMotor();
    }

    //Stops all flywheel motors
    public void FlywheelStop(){
        m_LeftShooterLeader.stopMotor();
        m_LeftFollowerShooter.stopMotor();
        m_RightShooterLeader.stopMotor();
        m_RightFollowerShooter.stopMotor();
    }

    //Stops only the hood motor
    public void HoodStop(){
        m_Hood.stopMotor();
    }
}
