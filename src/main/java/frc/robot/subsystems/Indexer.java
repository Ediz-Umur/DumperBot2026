package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase{
    //Object Creations
    private TalonFX m_IndexerMotor = new TalonFX(Constants.sIndexer.dIndexerMotorID);
    private TalonFXConfiguration m_IndexerConfiguration = new TalonFXConfiguration();
    private Slot0Configs m_IndexerPIDConfigs = m_IndexerConfiguration.Slot0;
    private MotionMagicConfigs m_IndexerMMConfigs = m_IndexerConfiguration.MotionMagic;

    public Indexer(){
        //Motor Configurations
        m_IndexerConfiguration.Feedback.SensorToMechanismRatio = Constants.sIndexer.cIndexerGearRatio;

        m_IndexerConfiguration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.sIndexer.cIndexerCurrentLimit));

        //PID Gains
        m_IndexerPIDConfigs.kS = Constants.sIndexer.pIndexerS;
        m_IndexerPIDConfigs.kA = Constants.sIndexer.pIndexerA;
        m_IndexerPIDConfigs.kV = Constants.sIndexer.pIndexerV;
        m_IndexerPIDConfigs.kP = Constants.sIndexer.pIndexerP;
        m_IndexerPIDConfigs.kI = Constants.sIndexer.pIndexerI;
        m_IndexerPIDConfigs.kD = Constants.sIndexer.pIndexerD;

        //MotionMagic Gains
        m_IndexerMMConfigs.MotionMagicAcceleration = Constants.sIndexer.mmIndexerAcceleration;
        m_IndexerMMConfigs.MotionMagicJerk = Constants.sIndexer.mmIndexerJerk;


        //Applying configurators
        m_IndexerMotor.getConfigurator().apply(m_IndexerConfiguration);
    }

    //Moving the indexer at a desired velocity
    public void IndexerMove(double Velocity){
        VelocityVoltage m_IndexerMotionMagic = new VelocityVoltage(Velocity).withEnableFOC(true);
        m_IndexerMotor.setControl(m_IndexerMotionMagic);
    }

    //Getting the front roller velocity
    public double IndexerVelocity(){
        return(m_IndexerMotor.getVelocity().getValueAsDouble());
    }

    //Stops all motors of the subsystem
    public void IndexerStop(){
        m_IndexerMotor.stopMotor();
    }
}
