package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limey extends BaseHardware {

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    private Limelight3A limelight;
    private LLResult result;

    private double tx = 0;
    private double ty = 0;
    private double tagDistance = 0;
    private double tagAngle = 0;
    private int tagID = -1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        limelight.setPollRateHz(100);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            var tag = result.getFiducialResults().get(0);

            tagID = tag.getFiducialId();
            tx = tag.getTargetXDegrees();
            ty = tag.getTargetYDegrees();

            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose != null) {
                tagDistance = pose.getPosition().z;
                tagAngle = pose.getOrientation().getYaw();
            }

            telemetry.addData("Limelight", "VALID TARGET");
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("ty", "%.2f°", ty);
            telemetry.addData("Distance", "%.2f", tagDistance);
            telemetry.addData("Yaw", "%.2f°", tagAngle);
            telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());

            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                telemetry.addData("Bot Pose (X,Y,Z)", "%.2f, %.2f, %.2f",
                        botPose.getPosition().x,
                        botPose.getPosition().y,
                        botPose.getPosition().z);
            }

        } else {
            tagID = -1;
            telemetry.addData("Limelight", "NO TARGET");

            if (result != null) {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());
            }
        }
    }

    @Override
    void stop() {

    }

    // Getters
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getTagID() { return tagID; }
    public double getTagDistance() { return tagDistance; }
    public double getTagAngle() { return tagAngle; }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setHardwareMap(HardwareMap hw) {
        this.hardwareMap = hw;
    }
}
