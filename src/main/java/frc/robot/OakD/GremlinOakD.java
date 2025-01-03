// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OakD;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commons.GeomUtil;

/** Add your docs here. */
public class GremlinOakD {
    private DoubleArraySubscriber xDistanceSubscriber;
    private DoubleArraySubscriber yDistanceSubscriber;
    private DoubleArraySubscriber zDistanceSubscriber;
    private DoubleArraySubscriber timestampSubscriber;

    private Pose3d camPose;
    private String name;
    private String path;

    private Supplier<Pose2d> robotPoseSupplier;

    public GremlinOakD(String name, Pose3d camPose, Supplier<Pose2d> poseSupplier){
        this.name = name;
        this.camPose = camPose;
        this.path = name;
        this.robotPoseSupplier = poseSupplier;

        this.xDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path  + "/xValues")
            .subscribe(new double[0]);
        this.yDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path + "/yValues")
            .subscribe(new double[0]);
        this.yDistanceSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path + "/zValues")
            .subscribe(new double[0]);  
        this.timestampSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(path + "/timestamps")
            .subscribe(new double[0]);
    }

    public ArrayList<TimestampedObjectPose> getCurrentEstimations(){
        ArrayList<TimestampedObjectPose> currentNotePoses = new ArrayList<>();
        double[] xDistances = xDistanceSubscriber.get();
        double[] yDistances = yDistanceSubscriber.get();
        double[] zDistances = zDistanceSubscriber.get();
        double[] timestamps = timestampSubscriber.get();

        if(xDistances.length != zDistances.length || xDistances.length != yDistances.length) return currentNotePoses;

        for(int i =0; i < xDistances.length; i++){
            Transform3d camToNote = new Transform3d(
                xDistances[i], 
                yDistances[i], 
                zDistances[i], 
                new Rotation3d());
            
            Pose3d camPoseField = new Pose3d(robotPoseSupplier.get()).transformBy(GeomUtil.pose3dToTransform3d(camPose).inverse());
            Pose3d notePoseField = camPoseField.transformBy(camToNote);
            currentNotePoses.add(new TimestampedObjectPose(notePoseField.toPose2d(), timestamps[i])); //Consider checking to make sure its near the ground
        }

        return currentNotePoses;
    }
    

}
