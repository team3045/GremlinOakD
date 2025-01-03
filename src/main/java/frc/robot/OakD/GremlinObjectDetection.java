// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OakD;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GremlinObjectDetection extends SubsystemBase {
  private GremlinOakD[] cameras;

  public ArrayList<TimestampedObjectPose> currentObjects;

  private static final double timeThreshold = 3; //Three seconds
  private static final double distanceTreshold = 0.5; //half meter
  public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);

  /** Creates a new GremlinObjectDetection. */
  public GremlinObjectDetection(GremlinOakD... cameras) {
    this.cameras = cameras;
  }

  public void updateWithValidEstimates(){
        ArrayList<TimestampedObjectPose> uncheckedUpdates = getRecentUpdates();
        
        for(TimestampedObjectPose possibleObject : uncheckedUpdates){
            //Check to make sure are inside the field
            Translation2d translation3d = possibleObject.pose().getTranslation();

            if (translation3d.getX() <= distanceTreshold
                || translation3d.getX() >= fieldSize.getX() - distanceTreshold
                || translation3d.getY() <= distanceTreshold
                || translation3d.getY() >= fieldSize.getY() - distanceTreshold) 
            {continue;}

            //Check to make sure they arent too close to any old notes
            //If they're close enough we're going to assume they are the same note and just update the position of the old one
            boolean isNearOldNote = false;
            for(int i = 0; i <currentObjects.size(); i++){
                if(isNearPose(possibleObject.pose(), currentObjects.get(i).pose(), distanceTreshold)){
                    currentObjects.set(i, possibleObject);
                    isNearOldNote = true;
                    break;
                }
            }

            if(!isNearOldNote) currentObjects.add(possibleObject);
        }
  }

  public ArrayList<TimestampedObjectPose> getRecentUpdates(){
    ArrayList<TimestampedObjectPose> recentUpdates = new ArrayList<>();

    for(GremlinOakD cam : cameras){
      recentUpdates.addAll(cam.getCurrentEstimations());
    }

    return recentUpdates;
  }

  public boolean isNearPose(Pose2d firstPose, Pose2d secondPose, double threshold){
        return firstPose.getTranslation().getDistance(secondPose.getTranslation()) > threshold;
    }

  @Override
  public void periodic() {
    //Clear out old object positions
    for(int i = 0; i < currentObjects.size(); i++){
      if(Timer.getFPGATimestamp() - currentObjects.get(i).timestamp() > timeThreshold)
        currentObjects.remove(i);
    }

    updateWithValidEstimates();
  }

  
}
