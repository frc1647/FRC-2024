package frc.robot.subsystems;

import static frc.robot.Constants.CameraConstants.kApriltagHeightDifferenceInches;
import static frc.robot.Constants.CameraConstants.kCameraAngle;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    
  PhotonCamera camera;
  PhotonTrackedTarget target;
  PhotonPipelineResult result;

  double yaw = 0;
  double pitch = 0;
  double area = 0;

  public Camera(){
    camera = new PhotonCamera("FrontCam");
    //camera = new PhotonCamera("photonvision");

    camera.setPipelineIndex(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    int tagID;
    if(result.hasTargets()){
      for (int i = 0; i < result.getTargets().size()-1; i++) {
        tagID = result.getTargets().get(i).getFiducialId();
        if (tagID == 4 || tagID == 7) {
          target = result.getTargets().get(i);
          yaw = target.getYaw();
          pitch = target.getPitch();
          area = target.getArea();
        }
      }

    
      SmartDashboard.putNumber("yaw", yaw);
      SmartDashboard.putNumber("pitch", pitch);
      SmartDashboard.putNumber("area", area);
      //SmartDashboard.putNumber("latency", latency());
    }
  }

  public boolean getHasTarget(){
    return result.hasTargets();
  }

  public double getYawAngle(){ //range: (_,_)
    if (result.hasTargets()) {
      return yaw;
    }else{
      return 0.0;
    }
  }

  public double getRange(){
    if (result.hasTargets()) {
      return kApriltagHeightDifferenceInches / Math.tan(Math.toRadians(kCameraAngle + pitch));
    }else{
      return 0.0;
    }
  }
  
  /*public double latency() {
    camera.setDriverMode(true);
    camera.setPipelineIndex(2);
    return result.getLatencyMillis() / 1000.0;
  }

  public void smartdashboard(){
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("area", area);
    SmartDashboard.putNumber("latency", latency());
  }*/
}
