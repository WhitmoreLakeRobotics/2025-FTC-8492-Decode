package org.firstinspires.ftc.teamcode.Hardware;

public class AutoAim {

    private final Limey limey;
    private final Turret turret;

    private boolean driverOverride = false;

    // Distance behind the tag to aim at
    // (your original comment preserved — but now using 8 inches)
    private static final double OFFSET_INCHES = 8.0;
    private static final double OFFSET = OFFSET_INCHES * 0.0254;  // convert to meters for botpose

    public AutoAim(Limey limey, Turret turret) {
        this.limey = limey;
        this.turret = turret;
    }

    public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }

    public double computeAimAngle() {

        // No tag detected
        if (limey.getTagID() == -1) {
            return Double.NaN;
        }

        // Get botpose from Limey
        double[] botpose = limey.getBotPose();
        if (botpose == null || botpose.length < 6) {
            return Double.NaN;
        }

        // Extract pose values
        double tagX = botpose[0];   // meters
        double tagY = botpose[1];   // meters
        double tagZ = botpose[2];   // meters
        double tagYawDeg = botpose[5];

        // Convert yaw to radians
        double tagYawRad = Math.toRadians(tagYawDeg);

        // Compute point behind the tag (8-inch offset)
        // Tag forward direction in robot space:
        //   forward = (sin(yaw), cos(yaw)) in X/Z plane
        double offsetX = -Math.sin(tagYawRad) * OFFSET;
        double offsetZ = -Math.cos(tagYawRad) * OFFSET;

        // Offset target point behind the tag
        double behindX = tagX + offsetX;
        double behindZ = tagZ + offsetZ;

        // Compute desired yaw angle
        double desiredYaw = Math.toDegrees(Math.atan2(behindX, behindZ));

        return desiredYaw;
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
