package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.RelativePose2d;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.RedTeamProp;

/**
 * Task to get the position of the red team prop.
 *
 * @author Lucas Bubner, 2024
 */
public class GetRedTeamPropTask extends ForeverTask {
    private final RedTeamProp redTeamProp;
    private volatile RelativePose2d position = RelativePose2d.LEFT;

    /**
     * Create a new GetRedTeamPropTask.
     *
     * @param redTeamProp the initialised and running RedTeamProp processor
     */
    public GetRedTeamPropTask(RedTeamProp redTeamProp) {
        this.redTeamProp = redTeamProp;
    }

    public RelativePose2d getPosition() {
        return position;
    }

    @Override
    protected void init() {
        if (!redTeamProp.isRunning()) {
            throw new IllegalStateException("RedTeamProp not attached and running on an active vision processor");
        }
    }

    @Override
    protected void periodic() {
        ContourData biggestContour = ContourData.getLargest(redTeamProp.getData());
        if (biggestContour != null) {
            Dbg.log(biggestContour.getYaw());
            position = biggestContour.getYaw() > 0.5 ? RelativePose2d.RIGHT : RelativePose2d.FORWARD;
        }
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
