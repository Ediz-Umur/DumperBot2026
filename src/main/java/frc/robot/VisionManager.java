package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionManager {
    public static Matrix <N3, N1> CalculateXYSTDDev(PoseEstimate poseEstimate){
        double multiTagBonus = 1;
        if(poseEstimate.tagCount > 2){
            multiTagBonus = 0.3;
        } else if(poseEstimate.tagCount > 1){
            multiTagBonus = 0.5;
        }
        //Calculate the value based off of the distance to the tag/s
        double calculatedXY = (Constants.sVision.pXYBase + (poseEstimate.avgTagDist * Constants.sVision.pXYScale)) * multiTagBonus;

        //Clamp the value
        double clampedXY = Math.max(Constants.sVision.pXYMin, Math.min(calculatedXY, Constants.sVision.pXYMax));

        //Return the STD with only the XY being valid
        return VecBuilder.fill(clampedXY, clampedXY, 999999);
    }

    public static Matrix <N3, N1> CalculateRotSTDDev(PoseEstimate poseEstimate){
        double multiTagBonus = 1;
        if(poseEstimate.tagCount > 2){
            multiTagBonus = 0.3;
        } else if(poseEstimate.tagCount > 1){
            multiTagBonus = 0.5;
        }
        //Calculate the value based off of the distance to the tag/s
        double calculatedRot = (Constants.sVision.pRotBase + (poseEstimate.avgTagDist * Constants.sVision.pRotScale)) * multiTagBonus;
        
        //Clamp the value
        double clampedRot = Math.max(Constants.sVision.pRotMin, Math.min(calculatedRot, Constants.sVision.pRotMax));

        //Return the STD with only the rotation being valid
        return VecBuilder.fill(999999, 999999, clampedRot);
    }

    public static boolean isTagRejected(PoseEstimate poseEstimate){
        //If the robot is turning too fast, reject measurement
        if (Math.abs(Units.radiansPerSecondToRotationsPerMinute(Constants.drivetrain.getState().Speeds.omegaRadiansPerSecond)) > Constants.sVision.rMaxTurnSpeed){
            return true;
        }

        //If the pose estimate doesnt exist, reject measurement
        if(poseEstimate.equals(null)){
            return true;
        }

        //If any of the tags have an ambiguity too high, reject measurement
        for(RawFiducial apriltag : poseEstimate.rawFiducials){
            if(apriltag.ambiguity > Constants.sVision.rMaxAmbiguity){
                return true;
            }
        }

        return false;
    }

    public static void OdometryInitialize(){
        for(String camera : Constants.sVision.cCameraNames){
            LimelightHelpers.SetIMUMode(camera, Constants.sVision.cCameraIMUMode);
        }
    }

    public static void OdometryCycle(){
        var driveState = Constants.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        for(String camera : Constants.sVision.cCameraNames){
            LimelightHelpers.SetRobotOrientation(camera, headingDeg, omegaRps, 0, 0, 0, 0);
            PoseEstimate Measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
            SmartDashboard.putBoolean(camera +" Translational Rejection: ", !isTagRejected(Measurement));
            if (!isTagRejected(Measurement)) {
                Constants.drivetrain.addVisionMeasurement(Measurement.pose, 
                                                          Measurement.timestampSeconds - ((Measurement.latency + LimelightHelpers.getLatency_Capture(camera)) / 1000), 
                                                          CalculateXYSTDDev(Measurement));
                                                  SmartDashboard.putNumber(camera + " X SYD", CalculateXYSTDDev(Measurement).get(0, 0));
                                                  SmartDashboard.putNumber(camera +" Y STD", CalculateXYSTDDev(Measurement).get(1, 0));
      }
      PoseEstimate MeasurementRotation = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);
      SmartDashboard.putBoolean(camera +" Rotational Rejection: ", !isTagRejected(MeasurementRotation));
      if (!isTagRejected(MeasurementRotation)){
        Constants.drivetrain.addVisionMeasurement(MeasurementRotation.pose,
                                                  MeasurementRotation.timestampSeconds - ((MeasurementRotation.latency + LimelightHelpers.getLatency_Capture(camera)) / 1000), 
                                                  CalculateRotSTDDev(MeasurementRotation));
                                                  SmartDashboard.putNumber(camera +" Rot STD", CalculateRotSTDDev(MeasurementRotation).get(2, 0));
      }

        }
    }
}
