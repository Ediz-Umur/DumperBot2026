package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    //Making the motor objects
    private TalonFX m_PivotMotor = new TalonFX(Constants.sIntake.dPivotID);
    private TalonFX m_RollerLeaderMotor = new TalonFX(Constants.sIntake.dRollerLeaderID);
    private TalonFX m_RollerFollowerMotor = new TalonFX(Constants.sIntake.dRollerFollowerID);


    //Making the configurator objects
    private TalonFXConfiguration m_PivotConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration m_RollerConfiguration = new TalonFXConfiguration();
    
    //Making the PID configurator objects
    private Slot0Configs m_PivotSlot0 = m_PivotConfiguration.Slot0;
    private Slot0Configs m_RollerSlot0 = m_RollerConfiguration.Slot0;

    //Making the Motion Magic configurator objects
    private MotionMagicConfigs m_PivotMotionConfigs = m_PivotConfiguration.MotionMagic;
    private MotionMagicConfigs m_RollerMotionConfigs = m_RollerConfiguration.MotionMagic;

    public Intake(){
        //Motor Configurations
        m_RollerConfiguration.Feedback.SensorToMechanismRatio = Constants.sIntake.cRollerGearRatio;
        m_RollerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_PivotConfiguration.Feedback.SensorToMechanismRatio = Constants.sIntake.cPivotGearRatio;
        m_PivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_PivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_PivotConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sIntake.cPivotCurrentLimit));
        m_RollerConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sIntake.cRollerCurrentLimit));

        //PID Configs
        m_PivotSlot0.GravityType = GravityTypeValue.Arm_Cosine;
        m_PivotSlot0.kS = Constants.sIntake.pPivotS;
        m_PivotSlot0.kA = Constants.sIntake.pPivotA;
        m_PivotSlot0.kV = Constants.sIntake.pPivotV;
        m_PivotSlot0.kG = Constants.sIntake.pPivotG;
        m_PivotSlot0.kP = Constants.sIntake.pPivotP;
        m_PivotSlot0.kI = Constants.sIntake.pPivotI;
        m_PivotSlot0.kD = Constants.sIntake.pPivotD;

        m_RollerSlot0.kS = Constants.sIntake.pRollerS;
        m_RollerSlot0.kA = Constants.sIntake.pRollerA;
        m_RollerSlot0.kV = Constants.sIntake.pRollerV;
        m_RollerSlot0.kP = Constants.sIntake.pRollerP;
        m_RollerSlot0.kI = Constants.sIntake.pRollerI;
        m_RollerSlot0.kD = Constants.sIntake.pRollerD;
    
        //MotionMagic Configs
        /*m_PivotMotionConfigs.MotionMagicCruiseVelocity = Constants.sIntake.mmPivotMMCruise;
        m_PivotMotionConfigs.MotionMagicExpo_kA = Constants.sIntake.mmPivotMMkA;
        m_PivotMotionConfigs.MotionMagicExpo_kV = Constants.sIntake.mmPivotMMkV;

        m_RollerMotionConfigs.MotionMagicAcceleration = Constants.sIntake.mmRollerAcceleration;
        m_RollerMotionConfigs.MotionMagicJerk = Constants.sIntake.mmRollerJerk;*/

        //Applying the configurations
        m_PivotMotor.getConfigurator().apply(m_PivotConfiguration);
        m_RollerLeaderMotor.getConfigurator().apply(m_RollerConfiguration);
    }

    //Moving the pivot based off of the amount of rotations the motor will do.
    public void GoToPosition(double rotations){
        MotionMagicExpoVoltage m_PivotMotionMagic = new MotionMagicExpoVoltage(rotations).withEnableFOC(true);
        m_PivotMotor.setControl(m_PivotMotionMagic);
    }
    
    //Moving the rollers at a desired rotation per second
    public void RollerMove(double RPS){
        VelocityVoltage m_RollerControl = new VelocityVoltage(RPS).withEnableFOC(true);
        m_RollerLeaderMotor.setControl(m_RollerControl);
        m_RollerFollowerMotor.setControl(new Follower(Constants.sIntake.dRollerLeaderID, MotorAlignmentValue.Opposed));
    }

    //Getting the pivot position
    public double GetPivotPosition(){
        return(m_PivotMotor.getPosition().getValueAsDouble());
    }

    //Getting the velocity of the roller
    public double GetRollerVelocity(){
        return(m_RollerLeaderMotor.getVelocity().getValueAsDouble());
    }

    //Stops all motors of the subsystem
    public void MotorStop(){
        m_PivotMotor.stopMotor();
        m_RollerLeaderMotor.stopMotor();
        m_RollerFollowerMotor.stopMotor();
    }

    public void RollerStop(){
        m_RollerLeaderMotor.stopMotor();
        m_RollerFollowerMotor.stopMotor();
    }
}
