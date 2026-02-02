package org.firstinspires.ftc.teamcode.aurumCode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class DesiredTag {
    private AprilTagDetection desiredTag;
    boolean targetFound;

    public int findPatternTag(List<AprilTagDetection> currentDetections){
        targetFound = false;
        desiredTag = null;
        int foundTagId = -1;

        for(AprilTagDetection detections : currentDetections){
            if(detections.metadata != null){
                if(detections.id == 20 || detections.id == 24){
                    targetFound = true;
                    desiredTag = detections;
                    foundTagId = detections.id;
                    break;
                }
            }
        }
        return foundTagId;
    }

    public boolean isTargetFound(){
        return targetFound;
    }
    public String getTagName(){
        if(desiredTag != null) return desiredTag.metadata.name;
        return "N/A";
    }
    public int getTagID(){
        if(desiredTag != null) return desiredTag.id;
        return 0;
    }
    public double getTagYaw(){
        if(desiredTag != null) return desiredTag.ftcPose.yaw;
        return 0.0;
    }
    public double getTagBearing(){
        if(desiredTag != null) return  desiredTag.ftcPose.bearing;
        return 0.0;
    }
    public double getTagRange(){
        if(desiredTag != null) return  desiredTag.ftcPose.range;
        return 0.0;
    }
    public double getTagZ(){
        if(desiredTag != null) return desiredTag.ftcPose.z;
        return 0.0;
    }
    public double getTagY(){
        if(desiredTag != null) return desiredTag.ftcPose.y;
        return 0.0;
    }
    public double getTagX(){
        if(desiredTag != null) return desiredTag.ftcPose.x;
        return 0.0;
    }
}
