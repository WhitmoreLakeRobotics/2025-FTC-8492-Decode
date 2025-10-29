
package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

//import android.graphics.Path;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Pedro Auton test", group = "Examples")
public class PedroAutontest extends OpMode {
    //Trying with panels inside class public PedroTelemetry pedroPanelsTelemetry = new PedroTelemetry();
    //public PedroDrawPath drawThis = new PedroDrawPath();
    private String thisUpdate = "7";
    private TelemetryManager telemetryMU;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime pTimer;// this is for pausing at the end of a path

    private int pathState;
    /* I found these poses wrong for this game so I've redone them to fit what I think it should be for a Goal start position  private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
      private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
      private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
      private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
      private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
  */
    //Thes are my adaptations
    private final Pose startPose = new Pose(22, 120, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(50, 75, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(24, 83, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setBrakingStrength(1);
        scorePreload.setBrakingStart(0);

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* return to start */
        //returnStart = follower.pathBuilder()
        //      .addPath(new BezierLine( scorePose, startPose))
        //    .setLinearHeadingInterpolation(scorePose.getHeading(),startPose.getHeading())
        //  .build();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
       upDatePanels();
        Drawing.drawDebug(follower);
        //  pedroPanelsTelemetry.loop();
        //  drawThis.loop();
     /*   telemetryMU.debug("x:" + myOTOS.getPosition().x);

        telemetryMU.debug("y:" + myOTOS.getPosition().y);
        telemetryMU.debug("heading:" + myOTOS.getPosition().h);
        //telemetryMU.debug("total heading:" + follower.getTotalHeading());
        telemetryMU.update(telemetry);*/
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pTimer = new ElapsedTime();

        follower = testChassisConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();
        setPathState(0);
        // disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
        // Feedback to Driver Hub for debugging
        telemetryMU.addData("path state", pathState);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("path", follower.getCurrentPath());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", follower.getPose().getHeading());
        telemetryMU.addData("pose", follower.poseTracker);
        telemetryMU.addData("pose history", scorePose);

        telemetryMU.update();
        Drawing.drawDebug(follower);


    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
public void upDatePanels(){
    telemetryMU.addData("Update ", thisUpdate);
    // Feedback to Driver Hub for debugging
    telemetryMU.addData("path state", pathState);
    telemetryMU.addData("x", follower.getPose().getX());
    telemetryMU.addData("y", follower.getPose().getY());
    telemetryMU.addData("heading Degrees:", Math.toDegrees(follower.getPose().getHeading()));
    //    telemetryMU.addData("path", follower.getCurrentPath());
    telemetryMU.addData("y", follower.getPose().getY());
    telemetryMU.addData("heading", follower.getPose().getHeading());

    telemetryMU.update();
}
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                // adding in a pause
                if (!follower.isBusy()) {
                    pTimer.reset();
                    while (pTimer.seconds() < 1.0) { //wait
                        telemetryMU.addData("Status", "waiting for 1 sec");
                        telemetryMU.addData("Elapsed:", pTimer.seconds());
                        upDatePanels();
                       // telemetryMU.update();
                    }
                    setPathState(-1);

                }
                break;


//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//
//
//     case 2:
////
//          /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//          if (!follower.isBusy()) {
//              /* Grab Sample */
////
//              /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//              follower.followPath(scorePickup1, true);
//              setPathState(3);
//          }
//          break;
//      case 3:
//          /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//          if (!follower.isBusy()) {
//              /* Score Sample */
////
//              /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//              follower.followPath(grabPickup2, true);
//              setPathState(4);
//          }
//          break;
//
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
        }
    }


    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        //       private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
/**

 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
// */
//private void DrawingAuton {
//    public static final double ROBOT_RADIUS = 9; // woah
//    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
//
//    private static final Style robotLook = new Style(
//            "", "#3F51B5", 0.75
//    );
//    private static final Style historyLook = new Style(
//            "", "#4CAF50", 0.75
//    );
//
//    /**
//     * This prepares Panels Field for using Pedro Offsets
//     */
//    public static void init() {
//        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
//    }
//
//    /**
//     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
//     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
//     *
//     * @param follower Pedro Follower instance.
//     */
//    public static void drawDebug(Follower follower) {
//        if (follower.getCurrentPathChain() != null && follower.getCurrentPath() != null) {
//            drawPath(follower.getCurrentPathChain(), robotLook);
//            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
//        } else if (follower.getCurrentPath() != null) {
//            drawPath(follower.getCurrentPath(), robotLook);
//            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
//        }
//
//        drawPoseHistory(follower.getPoseHistory(), historyLook);
//        drawRobot(follower.getPose(), historyLook);
//
//        sendPacket();
//    }
//
//    /**
//     * This draws a robot at a specified Pose with a specified
//     * look. The heading is represented as a line.
//     *
//     * @param pose  the Pose to draw the robot at
//     * @param style the parameters used to draw the robot with
//     */
//    public static void drawRobot(Pose pose, Style style) {
//        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
//            return;
//        }
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(pose.getX(), pose.getY());
//        panelsField.circle(ROBOT_RADIUS);
//
//        Vector v = pose.getHeadingAsUnitVector();
//        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
//        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
//        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
//
//        panelsField.setStyle(style);
//        panelsField.moveCursor(x1, y1);
//        panelsField.line(x2, y2);
//    }
//
//    /**
//     * This draws a robot at a specified Pose. The heading is represented as a line.
//     *
//     * @param pose the Pose to draw the robot at
//     */
//    public static void drawRobot(Pose pose) {
//        drawRobot(pose, robotLook);
//    }
//
//    /**
//     * This draws a Path with a specified look.
//     *
//     * @param path  the Path to draw
//     * @param style the parameters used to draw the Path with
//     */
//    public static void drawPath(Path path, Style style) {
//        double[][] points = path.getPanelsDrawingPoints();
//
//        for (int i = 0; i < points[0].length; i++) {
//            for (int j = 0; j < points.length; j++) {
//                if (Double.isNaN(points[j][i])) {
//                    points[j][i] = 0;
//                }
//            }
//        }
//
//        panelsField.setStyle(style);
//        for (int i = 0; i < points[0].length - 1; i++) {
//            panelsField.moveCursor(points[0][i], points[1][i]);
//            panelsField.line(points[0][i + 1], points[1][i + 1]);
//        }
//    }
//
//    /**
//     * This draws all the Paths in a PathChain with a
//     * specified look.
//     *
//     * @param pathChain the PathChain to draw
//     * @param style     the parameters used to draw the PathChain with
//     */
//    public static void drawPath(PathChain pathChain, Style style) {
//        for (int i = 0; i < pathChain.size(); i++) {
//            drawPath(pathChain.getPath(i), style);
//        }
//    }
//
//    /**
//     * This draws the pose history of the robot.
//     *
//     * @param poseTracker the PoseHistory to get the pose history from
//     * @param style       the parameters used to draw the pose history with
//     */
//    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
//        panelsField.setStyle(style);
//
//        int size = poseTracker.getXPositionsArray().length;
//        for (int i = 0; i < size - 1; i++) {
//
//            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
//            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
//        }
//    }
//
//    /**
//     * This draws the pose history of the robot.
//     *
//     * @param poseTracker the PoseHistory to get the pose history from
//     */
//    public static void drawPoseHistory(PoseHistory poseTracker) {
//        drawPoseHistory(poseTracker, historyLook);
//    }
//
//    /**
//     * This tries to send the current packet to FTControl Panels.
//     */
//    public static void sendPacket() {
//        panelsField.update();
//    }
