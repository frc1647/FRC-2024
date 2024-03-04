public class Blind extends Command{
  intake intake;
  CANDrivetrain drivetrain;
  LaunchNote launchNote;
  PrepareLaunch prepareLaunch;
  Camera camera;
  public Blind(intake m_intake, CANDrivetrain m_drivetrain, LaunchNote m_launchNote, PrepareLaunch m_prepareLaunch, Camera m_camera){
    m_intake = intake;
    m_drivetrain = drivetrain;
    m_camera = camera;
    addRequirements(intake, drivetrain, camera);
  }
  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      int tag = 0
      for (int i = 0; i < camera.aprilTagFieldLayout().size(); i++) {
        if (camera.robotPose() == camera.aprilTagFieldLayout().index(i)){
          tag = i
          //break out of this loop
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(camera.getHasTarget()){
            drivetrain.rawTankDrive(camera.getYawAngle() * lookAtApriltagConstants.kP, -camera.getYawAngle() * lookAtApriltagConstants.kP);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
    }
}
