public class Camera{
PhotonCamera camera = new PhotonCamera("photonvision");
var result = camera.getLatestResult();
	PhotonTrackedTarget target
	double yaw
	double pitch
	double area
	double skew
	public void objectData{
if(result.hasTargets()){
yaw = target.getYaw();
pitch = target.getPitch();
area = target.getArea();
skew = target.getSkew();		
}
}
int targetID
double poseAmbiguity
Pose2D robotPose
public void aprilTagData{
try{
targetID = target.getFiducialId();
poseAmbiguity = target.getPoseAmbiguity();
robotPose = PhotonUtils.estimateFieldToRobot(
  kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);
}finally{
	
}
	}
	double distanceToTarget
	double range
	Translation2d translation
	Rotation2d targetYaw
	public void distances{
		if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var resu	lt = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
			translation = PhotonUtils.estimateCameraToTargetTranslation(
  distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));
			targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);
		AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	}
	
	AprilTagFieldLayout aprilTagFieldLayout
	public void fieldLayout{
		aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	}
	double latencySeconds
	public void latency{
		camera.setDriverMode(true);
		camera.setPipelineIndex(2);
		latencySeconds = result.getLatencyMillis() / 1000.0;
	}
}
