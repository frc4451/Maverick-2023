package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class Photoner {
    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    public Photoner(String name) {
        this.camera = new PhotonCamera(name);
    }

    public void updateResult() {
        this.result = this.camera.getLatestResult();
    }

    public PhotonPipelineResult getResult() {
        return this.result;
    }

    public PhotonCamera getCamera() {
        return this.camera;
    }

    /**
     * Output for turning to best target with offset
     * 
     * @param tagOffsetMeters Offset from AprilTag in meters
     */
    public double getTurnTo(double tagOffsetMeters) {
        final PhotonTrackedTarget target = this.result.getBestTarget();
        if (target == null) {
            return 0;
        }

        final Transform3d targetPose = target.getBestCameraToTarget();

        double error = -(targetPose.getY() + tagOffsetMeters);
        if (this.result.hasTargets() && (Math.abs(error) >= 0.1)) {
            return (error * Constants.DT_Settings.TARGETING_TURNING_PG)
                    + (Math.signum(error) * Constants.DT_Settings.TARGETING_TURNING_SG);
        } else {
            return 0;
        }
    }
}