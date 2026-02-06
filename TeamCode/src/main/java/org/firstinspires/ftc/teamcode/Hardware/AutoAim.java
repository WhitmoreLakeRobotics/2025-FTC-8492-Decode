package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;


public class AutoAim {

    private final Limey limey;
    private final Turret turret;
    private final DriveTrain driveTrain;   // MJD — added so we can read robot heading

    private boolean driverOverride = true;

    // Distance behind the tag to aim at
    private static final double OFFSET_INCHES = 8.0;
    private static final double OFFSET = OFFSET_INCHES * 0.0254;  // convert to meters for botpose

    public AutoAim(Limey limey, Turret turret, DriveTrain driveTrain) {
        this.limey = limey;
        this.turret = turret;
        this.driveTrain = driveTrain;   // MJD — store drivetrain reference
    }

    public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }

    public double computeAimAngle() {

        // No tag detected
        if (limey.getTagID() == -1) {
            return Double.NaN;
        }

        // Horizontal angle to tag (degrees)
        double tx = limey.getTx();
        if (Double.isNaN(tx)) {
            return Double.NaN;
        }

        // Vertical angle to tag (degrees)
        double ty = limey.getTy();
        if (Double.isNaN(ty)) {
            return Double.NaN;
        }
        // distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
        // You MUST set these to match your robot:
        double cameraHeight = 11.0;      // inches — adjust for your robot
        double targetHeight = 14.375;    // inches — FTC backdrop tag height
        double cameraAngle = 25.0;       // degrees — adjust for your mount

        double cameraAngleRad = Math.toRadians(cameraAngle + ty);
        double distanceInches = (targetHeight - cameraHeight) / Math.tan(cameraAngleRad);

        // offsetAngle = atan(offset / distance)
        double offsetAngleDeg = Math.toDegrees(Math.atan(OFFSET_INCHES / distanceInches));

        // If we want to aim BEHIND the tag, we subtract the correction
        double correctedTx = tx - offsetAngleDeg;

        double robotHeading = driveTrain.getCurrentHeading();
        double desiredHeading = robotHeading + correctedTx;

        // Normalize to [-180, 180]
        desiredHeading = ((desiredHeading + 540) % 360) - 180;

        return desiredHeading;
    }

    // AutoAim drives the turret ONLY when override is off.
    public void update() {

        // If driver override OR turret is missing, do nothing
        if (driverOverride || turret == null) {
            return;
        }

        double angle = computeAimAngle();
        if (Double.isNaN(angle)) return;

        turret.setTargetAngle(angle);
    }

}
