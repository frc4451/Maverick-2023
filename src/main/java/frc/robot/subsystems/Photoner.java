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
     * @param horizonalOffsetMeters Horizontal offset from AprilTag in meters.
     *                              The exact point this is offset from depends on
     *                              your PhotonVision configuration in the UI.
     */
    public double getTurnTo(double horizonalOffsetMeters) {
        final PhotonTrackedTarget target = this.result.getBestTarget();

        // If we don't have this null check the code'll crash when there isn't a target
        if (target == null) {
            return 0;
        }

        final Transform3d targetPose = target.getBestCameraToTarget();

        final double error = -(targetPose.getY() + horizonalOffsetMeters);
        // We shouldn't need to to check if there's any targets here because if there
        // weren't targets we would've early returned.
        if (Math.abs(error) >= 0.1) {
            return (error * Constants.DT_Settings.TARGETING_TURNING_PG)
                    + (Math.signum(error) * Constants.DT_Settings.TARGETING_TURNING_SG);
        } else {
            return 0;
        }
    }
}
