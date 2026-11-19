package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Vision{

    private static AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private static PhotonCamera cmrTavuk = new PhotonCamera(Constants.sVision.cCameraNames[0]);
    private static PhotonCamera cmrKarga = new PhotonCamera(Constants.sVision.cCameraNames[1]);
    private static PhotonCamera cmrOrdek = new PhotonCamera(Constants.sVision.cCameraNames[2]);


    private static PhotonPoseEstimator estTavuk = new PhotonPoseEstimator(FieldLayout, Constants.sVision.cRobotToCameraPositions[0]);
    private static PhotonPoseEstimator estKarga = new PhotonPoseEstimator(FieldLayout, Constants.sVision.cRobotToCameraPositions[1]);
    private static PhotonPoseEstimator estOrdek = new PhotonPoseEstimator(FieldLayout, Constants.sVision.cRobotToCameraPositions[2]);

            
    static PhotonCamera[] cameras = {cmrTavuk, cmrKarga, cmrOrdek};
    static PhotonPoseEstimator[] poseEstimators = {estTavuk, estKarga, estOrdek};
            
    private static Matrix<N3, N1> CurrentSTDDevs;
                    
    public Vision(){
                
    }
    
    public static void OdometryCycle(){
        for(int i = 0; i < cameras.length; i++){
            getVisionEstimates(i);
        }
    }   

    public static void getVisionEstimates(int cameraNum){
        PhotonCamera camera = cameras[cameraNum];
        PhotonPoseEstimator poseEstimator = poseEstimators[cameraNum];
        Optional<EstimatedRobotPose> visEst = Optional.empty();
        for(PhotonPipelineResult result : camera.getAllUnreadResults()){
            visEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visEst.isEmpty()){
                visEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            setVisionSTDDevs(visEst, result.getTargets(), poseEstimator);
            visEst.ifPresent(est -> {
            Constants.drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), result.getTimestampSeconds(), getCurrentStdDevs());
        });
    }
}
                                                
    private static void setVisionSTDDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator){
        boolean deletePose = false;
        if (estimatedPose.isEmpty()){
            CurrentSTDDevs = Constants.sVision.pSingleTag;
        } else {
            Matrix<N3, N1> estStdDevs = Constants.sVision.pSingleTag;
            int targetNum = 0;
            double avgDist = 0;                                
            
            //If the drivetrain is rotating too fast, reject the measurement
            if(Math.abs(Units.radiansPerSecondToRotationsPerMinute(Constants.drivetrain.getState().Speeds.omegaRadiansPerSecond)) > Constants.sVision.rMaxTurnSpeed) deletePose = true;

            for (PhotonTrackedTarget trg : targets){
                if (trg.poseAmbiguity > Constants.sVision.pMaxAmbiguity) deletePose = true;
                Optional<Pose3d> TagPose = poseEstimator.getFieldTags().getTagPose(trg.fiducialId);
                                
                if (TagPose.isEmpty()) continue;
                targetNum ++;
                avgDist += TagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }
                                
            if (targetNum == 0){
                CurrentSTDDevs = Constants.sVision.pSingleTag;
            } else {
                avgDist /= targetNum;
                                
                if (targetNum > 1) CurrentSTDDevs = Constants.sVision.pMultitag;
                if (targetNum == 1 && avgDist > Constants.sVision.pAvgMaxDistance)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * (avgDist / Constants.sVision.pStdPenalty)));
                                
                if (!deletePose) CurrentSTDDevs = estStdDevs;
                else CurrentSTDDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        }
    }                

    public static Matrix <N3, N1> getCurrentStdDevs(){
        return CurrentSTDDevs;
    }

    public static int getCameraCount(){
        return cameras.length;
    }
}
