package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AutoAim {
    //auto aim for turret

    public Limey limey;
    public Turret turret;
    private LLResult result;

    private static final double OFFSET = 0.2032; // 8 inches in meters

      private boolean driverOverride = false;

    public AutoAim(Limey limey, Turret turret) {
        this.limey = limey;
        this.turret = turret;
    }

        public void AutoAim(Limey limey, Turret turret) {
        this.limey = limey;
        this.turret = turret;
    }
        public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }

    public void update() {

        // if driver is manually steering turret, do nothing
        if (driverOverride) return;

        // no tag? do nothing
        if(limey.getTagID() == -1)return;

        // Limelight botpose array:
        // [0]=x,[1]=y,[2]=z,[3]=roll[4]=pitch,[5]=yaw
        double[] botpose = limey.getBotPose();
        if (botpose == null || botpose.length < 6) return;

        double tagX = botpose[0];
        double tagY = botpose[1];
        double tagZ = botpose[2];
        double tagYawDeg = botpose[5];

        //Convert yaw to radians
        double tagYawRad = Math.toRadians(tagYawDeg);

        //Compute the point 8 inches BEHIND the tag
        //Behind = tag position - fowardDirection  OFFSET
        double behindX = tagX - Math.sin(tagYawRad) * OFFSET;
        double behindZ = tagZ -  Math.cos(tagYawRad) * OFFSET;

        //Turret jis assumed to be at (0,0) in robot space
        //Compute yaw to the "behind" point
        double desiredYaw = Math.toDegrees(Math.atan2(behindX, behindZ));

        //Send angle to turret PID
        turret.setTargetAngle(desiredYaw);
    }
}
