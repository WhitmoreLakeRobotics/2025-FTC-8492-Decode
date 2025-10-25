package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.*;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
public class PedroTelemetry extends OpMode {

    private TelemetryManager telemetryMU;
    //SparkFunOTOS myOTOS = new SparkFunOTOS("Otto");
    private SparkFunOTOS myOTOS;

    @Override
    public void init() {
        myOTOS = hardwareMap.get(SparkFunOTOS.class, "otto");
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

        //       cmdUpdatePanels();
    }

    //   This was copied from Tuning as an example
    //   public void cmdUpdatePanles(double x, double y, double Head, double TotalHead)
  //  public void cmdUpdatePanels(SparkFunOTOS.Pose2D InPose);
 /*
    telemetryMU.debug("x:" + x);
    telemetryMU.debug("y:" + InPose.getY());
    telemetryMU.debug("heading:" + follower.getPose().getHeading());
    telemetryMU.debug("total heading:" + follower.getTotalHeading());
    telemetryMU.update(telemetry);

/*
telemetryM.  debug("x:" + follower.getPose().getX());
telemetryM.debug("y:" + follower.getPose().getY());
telemetryM.debug("heading:" + follower.getPose().getHeading());
telemetryM.debug("total heading:" + follower.getTotalHeading());
telemetryM.update(telemetry);

    draw();
 */


    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

        //InPose = SparkFunOTOS.Pose2D;
        telemetryMU.debug("x:" + myOTOS.getPosition().x);

        telemetryMU.debug("y:" + myOTOS.getPosition().y);
        telemetryMU.debug("heading:" + myOTOS.getPosition().h);
        //telemetryMU.debug("total heading:" + follower.getTotalHeading());
        telemetryMU.update(telemetry);

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        super.stop();
    }
}




