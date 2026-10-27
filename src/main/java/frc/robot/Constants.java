package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/*
     * Rules & Syntax for the Constants Folder
     *
     * RULES
     * PID Settings
     * 
     * P, I and D are the main things to change, 
     * since all of the other constants should not be changed
     * 
     * P is the proportional value for the error movement, so it grabs P percent (1 is 100%)
     * of the error to move. This should be the main way to change PID.
     * I is the integral, which just adds up all previous error then adds I percent (1 is 100%)
     * This is usually the last thing to change.
     * D is the derivative, which uses the derivative of the error, at D percent (1 is 100%)
     * This is the second best way to tune PID, and intuitively, makes the movement rate match the desired rate of change
     * 
     * S is the voltage to overcome friction
     * A is the amount of voltage needed to set acceleration to 1 m/s^2
     * V is the amount of voltage needed to set velocity to 1 m/s
     * G is the amount of voltage needed to negate the gravitational pull
     * 
     * 
     * Motion Magic Settings
     * 
     * Motion Magic is used to make the desired movement smoother, to ensure that 
     * there is less erratic movement
     * 
     * There are two types of motion magic: Position, which has a specific position (in rotations) to go, and Velocity, which has a set velocity goal (in rotations/second). 
     * All position motion magic uses MotionMagicExpoTorqueFOC
     * All velocity motion magic uses MotionMagicVelocityTorqueFOC
     * 
     * For position, there are 3 values:
     * Cruise, which is the maximum velocity of the profile during the movement (This can be set to 0 for unlimited speed cap)
     * kA, which is the voltage needed to get 1 rotations/second^2
     * kV, which is the voltage needed to get 1 rotations/second
     * 
     * For velocity, there are 2 values:
     * Acceleration, which is the final acceleration goal to reach
     * Jerk, which is how fast the mechanism will reach the max acceleration. But, this is not in seconds. 
     * Instead, think of it like   "The amount of acceleration added per second", so a jerk 10x the acceleration
     *(the default jerk), means that the max acceleration will be reached in 0.1 seconds, since you are
     * "adding" 10x the max acceleration per second.
     * 
     * 
     * As long as the robot or the electronics do not change, don't touch any device IDs
     * Any other miscellanious constants have their units and descriptions written on them.
     * 
     * OTHER COMMON WORDS:
     * GearRatio
     * 
     * The ratio of movement between the motor and mechanism, motor and encoder, or encoder and mechanism
     * Simply, it is the number of times the motor / encoder have to rotate for the encoder / mechanism to fully rotate once. Numbers higher than 1 is a reduction, smaller is a step-up
     * If there is no encoder, the GR is always Motor --> Mechanism and the value will simple say (___GearRatio)
     * If there is an encoder, there are two GRs: Motor --> Encoder (___GearRatioMtE) and Encoder --> Mechanism (___GearRatioEtMc)
     * 
     * CurrentLimit
     * The maximum amount of voltage that can go to the piece in question, in Volts
     * 
     * Tolerance
     * The amount of error (±) that the code can accept away from the target speed to consider it a valid value (speed or position)
     * 
     * SYNTAX
     * 
     * Lowercase items are the objects created by the folder
     * "s" are for the classes, or "subsystems", of the constants/mechanisms
     * "d" is for Device ID
     * "f" is for CANCoder gains
     * "r" is for CANRange gains
     * "p" is for PID gains
     * "mm" is for Motion Magic gains
     * "i" is for SysID gains
     * "s" (in a class) is for state-space gains (only for shooter)
     * "c" is for other motor configs
     * "t" is for tolerances
     * "k" is for miscellanious constants
     * If there is no letter at the start (like the limelight constants), it is because there is no good way to separate them. 
     * 
     * There is a separate naming scheme for the Vision system specifically, since its so differentİ
     * "d" is for stardard deviation calculations
     * "c" is for camera specifics
     * "r" is for Apriltag rejection
     * 
     * Every word in a constant starts with a capital letter and acronyms have all letters capitalized.
     */
public class Constants {

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Intake intake = new Intake(); 
    public static final Indexer indexer = new Indexer();
    public static final Shooter shooter = new Shooter();

    public static boolean blueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

    public class sVision{
        //Camera Specifics
        public static final String[] cCameraNames = {"limelight-tavuk", "limelight-karga", "limelight-ordek"};
        public static final int cCameraIMUMode = 0;

        //Pose Estimator
        public static final double pXYBase = 0.3; //Meters | The base std value for the translation
        public static final double pRotBase = 0.65; //Degrees | The base std value for the rotation

        public static final double pXYScale = 0.2;  //Meters | How much to increase the std dev by per meter of tag distance
        public static final double pRotScale = 0.2; //Degrees | How much to increase the std dev by per meter of tag distance

        public static final double pXYMin = 0.15; //The minimum std value for the translation
        public static final double pXYMax = 1; //The maximum std value for the translation
        public static final double pRotMin = 0.3; //The minimum std value for the rotation
        public static final double pRotMax = 1.75; //The maximum std value for the rotation

        //Apriltag Rejection
        public static final double rMaxTurnSpeed = 30; //RotationsPerMinute
        public static final double rMaxAmbiguity = 0.9; //Percent (1 is 100%) | The maximum amount of ambiguity that is accepted
    }

    public class sDrivetrain{
        //General ID
        public static final int dGyroID = 13;

        //General
        public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        //HubAlign
        public static final double tMaxTimer = 0.5; //The maximum amount of time the command can run
        public static final double tCounterAmount = 15; //The number that the counter has to reach to consider the command done
        public static final double tDegreeTolerance = 1; //Degrees | The ± degree tolerance given to the drivetrain angle

        public static final double pHubAlignP = 0.150;
        public static final double pHubAlignI = 0.0;
        public static final double pHubAlignD = 0.01;

        //XLock
        public static final double kXLockDisableLimit = 0.5; //Percent movement (1 is max) | How much the joystick has to go away from the center for XLock to be disabled
    
    }

    public class sFieldElement{
        public static final double kHubHeight = 1.8288;

        public static final Translation2d kBlueHubPosition = new Translation2d(4.625594, 4.034536);
        public static final Translation2d kRedHubPosition = new Translation2d(11.915394, 4.034536);

        public static final Translation2d kFieldSize = new Translation2d(16.5410134,8.069326);
        //NOTE!! THIS IS THE BOTTOM TARGET ON THE BLUE ALLIANCE, EVERY OTHER TARGET IS DERIVED FROM THIS
        //SECOND NOTE!! THIS IS IN FIELD COORDINATE SYSTEM, SO CONSIDER THE ORIGIN AS THE BOTTOM OF THE BLUE ALLIANCE WALL WHEN BLUE IS AT THE LEFT SIDE
        public static final Translation2d kFeedTarget = new Translation2d(1, 1.5);
    }

    public class sIntake{
        //Device IDs   
        public static final int dRollerLeaderID = 20;
        public static final int dRollerFollowerID = 21;
        public static final int dPivotID = 22;

        //PID Gains
        public static final double pRollerS = 0;
        public static final double pRollerA = 0;
        public static final double pRollerV = 0;
        public static final double pRollerP = 50;
        public static final double pRollerI = 0;
        public static final double pRollerD = 0;

        public static final double pPivotS = 0;
        public static final double pPivotA = 0;
        public static final double pPivotV = 0;
        public static final double pPivotG = 0;
        public static final double pPivotP = 5;
        public static final double pPivotI = 0;
        public static final double pPivotD = 0;

        //Motor Configs
        public static final double cRollerGearRatio = 1.5;
        public static final double cPivotGearRatio = 0.111;
        
        public static final int cRollerCurrentLimit = 50;
        public static final int cPivotCurrentLimit = 50;

        //Tolerances

        public static final double tRollerTolerance = 1; //RotationsPerSecond
        public static final double tPivotTolerance = 5; //Rotations

        //Misc
        public static final double kPivotOpenPosition = 260; //Rotations | The encoder rotation value when the intake is OPEN
        public static final double kPivotClosedPosition = 35; //Rotations | The encoder rotation value when the intake is CLOSED //TODO
        public static final double kRollerSpeed = 65; //RotationsPerSecond

        public static final double kOscillateWaitTime = 1.5; //Seconds | How much time the IntakeOScillate command waits before starting in the Shoot command

    }
    public class sIndexer{
        //Devide IDs
        public static final int dIndexerMotorID = 30;

        //PID
        public static final double pIndexerS = 0;
        public static final double pIndexerA = 0;
        public static final double pIndexerV = 0;
        public static final double pIndexerP = 50;
        public static final double pIndexerI = 0;
        public static final double pIndexerD = 0;

        //MotionMagic 
        public static final double mmIndexerAcceleration = 100; //TODO
        public static final double mmIndexerJerk = 1000; //TODO

        //Motor Configs
        public static final double cIndexerGearRatio = 1.5;
        public static final int cIndexerCurrentLimit = 50; //TODO

        //Tolerances
        public static final double tIndexerTolerance = 1; //RotationsPerSecond //TODO

        //Misc
        public static final double kIndexerSpeed = 70; //RotationsPerSecond //TODO
    }
    public class sShooter{
        //Devide IDs
        public static final int dLeftShooterLeaderID = 40;
        public static final int dLeftShooterFollowerID = 41; 
        public static final int dRightShooterLeaderID = 42;
        public static final int dRightShooterFollowerID = 43;
        public static final int dKickerID = 44;
        public static final int dHoodID = 45;  
        public static final int dHoodEncoder = 46;
        public static final int dRangeLeft = 47;
        public static final int dRangeRight = 48;

        //CANCoder Configs
        public static final double fMagnetOffset = 0; //TODO
        public static final double fDiscontinuityPoint = 0; //TODO
        public static final SensorDirectionValue fSensorDirection = SensorDirectionValue.CounterClockwise_Positive; //TODO

        //CANRange Configs
        public static final double rDetectionRange = 0.1; //Meters | The treshold for the detection range, any distance below this is considered "detected"
        public static final double rMinSignalStrength = 2500; //The minimum strength allowed for CANRange value to be valid. Set this to the lowest value seen when a valid detection is up

        //PID Gains
        public static final double cFlywheelS = 0.0235;
        public static final double cFlywheelA = 0.008;
        public static final double cFlywheelV = 0.185;
        public static final double cFlywheelP = 6.7;
        public static final double cFlywheelI = 0;
        public static final double cFlywheelD = 0;

        public static final double cKickerkS = 0; 
        public static final double cKickerkA = 0; 
        public static final double cKickerkV = 0; 
        public static final double cKickerkP = 50; 
        public static final double cKickerkI = 0; 
        public static final double cKickerkD = 0; 

        public static final double cHoodG = 0;
        public static final double cHoodS = 0.03475;
        public static final double cHoodA = 0;
        public static final double cHoodV = 0;
        public static final double cHoodP = 7;
        public static final double cHoodI = 0;
        public static final double cHoodD = 0.1;

        //MotionMagic Gains
        public static final double mmFlywheelAcceleration = 100; //TODO
        public static final double mmFlywheelJerk = 1000; //TODO

        public static final double mmKickerAcceleration = 100; //TODO 
        public static final double mmKickerJerk = 1000; //TODO

        public static final double mmHoodCruise = 115.0; //TODO
        public static final double mmHoodkA = 0.01; //TODO
        public static final double mmHoodkV = 0.01; //TODO

        //Motor Configs
        public static final double cFlywheelGearRatio = 1.5;
        public static final double cHoodGearRatioMtE = 6.006;
        public static final double cHoodGearRatioEtMc = 0.5;
        public static final double cKickerGearRatio = 1.5; 

        public static final double cFlywheelCurrentLimit = 85;
        public static final double cHoodCurrentLimit = 50; //TODO
        public static final double cKickerCurrentLimit = 50; //TODO

        public static final InvertedValue cLeftFlywheelModuleInverted = InvertedValue.Clockwise_Positive; //TODO
        public static final InvertedValue cRightFlywheelModuleInverted = InvertedValue.CounterClockwise_Positive; //TODO

        //Tolerances
        public static final double tFlywheelTolerance = 5; //RotationsPerSecond //TODO
        public static final double tHoodTolerance = 1; //Rotations
        public static final double tKickerTolerance = 5; //RotationsPerSecond //TODO

        //Misc
        public static final double kKickerSpeed = 70; //RotationsPerSecond

        public static final double kFlywheelMaxSpeed = 100; //RotationPerSecond //TODO
        public static final double kFlywheelFeedSpeed = 40; //RotationsPerSecond | The speed of the flywheel while feeding
        
        public static final double kHoodMinAngle = 10; //Degrees
        public static final double kHoodMaxAngle = 30; //Degrees
        public static final double kHoodFeedAngle = 15;//Degrees
        public static final double kHoodClosedAngle = 5; //Degrees | The angle of the hood when it is not in use //TODO

        public static final double kFlywheelMult = 1.25; //The multiplier on the distance calculation
        public static final double kFlywheelRadius = 0.04; //Meters
        
        public static final double kFlywheelHeight = 0.404; //Meters
        public static final Translation2d kFlywheelOffsetFromCenter = new Translation2d(0, 0.335);
        public static final double kIdealAngle = -75; //Degrees | The ideal angle for the ball to go into the hub, with respect to the ground

        public static final double kTeleopMaxShootTime = 5; //Seconds | The maximum amount of time the shooter waits for balls to come in before ending the command in teleop
        public static final double kAutoMaxShootTime = 5; //Seconds | The maximum amount of time the shooter waits for balls to come in before ending the command in autonomous
        public static final double kOscillateDelayTime = 1; //Seconds | How much the command waits before starting the IntakeOscillate command suring the Shoto command

        public static final double kHoodDegreeToRotation = 0.302; //RotationPerDegree | How many rotations is 1 degree //TODO

        public static final double kWarmupMaxDistance = 2; //Meters | The maximum distance the warup command will activate in
        public static final double kWarmupScale = 0.2; //Percentage (1 is 100%) How many percent the flywheel will warmup in, with respect to the calculated speed in the Shoot command
    }
}
