package org.firstinspires.ftc.teamcode.Autons;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.testChassisConstants;




//package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

//import android.graphics.Path;


    @Autonomous(name = "Red Near Auton", group = "Examples")
    public class RedNearAuton extends OpMode {
        // inserting standard  WL auton stuff
        //RobotComp robot = new RobotComp();
        Robot robot = new Robot();

        private String thisUpdate = "3";
        private stage currentStage = stage._unknown;

        private enum stage {
            _unknown,
            _00_preStart,
            _10_Drive_Out,
            _20_LaunchPreload,
/*            _20_Strafe_Right,
            _30_Drive_Forward,
            _40_Strafe_Left,
            _50_Drive_Forward,
            _60_Strafe_Right,
            _70_Drive_Backward,
            _80_Strafe_Left,
            _90_Drive_Backward,
            _100_Strafe_Right,
            _110_Drive_Backward,*/
            _120_End;


        }
        private String RTAG = "8492-Auton";

        /* Declare OpMode members. */
        private ElapsedTime runtime = new ElapsedTime();
        //Trying with panels inside class public PedroTelemetry pedroPanelsTelemetry = new PedroTelemetry();
        //public PedroDrawPath drawThis = new PedroDrawPath();

        private TelemetryManager telemetryMU;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        private ElapsedTime pTimer;// this is for pausing at the end of a path

        private int pathState;

        private final Pose startPose = new Pose(130, 120, Math.toRadians(0)); // Start Pose of our robot.
        private final Pose launchPose = new Pose(84, 132, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        //private final Pose movePose = new Pose(, , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
        //  private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
        //  private final Pose pickup3Pose = new Pose(24, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

        private Path scorePreload;
        //Pathchain is used to draw the path the robot is supposed to follow.
        // only include the paths relevant to this auton
        private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

        public void buildPaths() {
            /* Build all your paths here.
            This is our scorePreload path. It goes from our start pose to the launch pose
             We are using a BezierLine, which is a straight line. */
            scorePreload = new Path(new BezierLine(startPose, launchPose));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());
            scorePreload.setBrakingStrength(5);
            scorePreload.setBrakingStart(2);


            /* create a path for each move segment
            This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            /*grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(launchPose, movePose))
                    .setLinearHeadingInterpolation(launchPose.getHeading(), movePose.getHeading())
                    .build();
*/
        }
        /**
         * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
         **/
        @Override
        public void loop() {

            // These loop the movements of the robot, these must be called continuously in order to work
            robot.loop();

            switch (currentStage) {
                case _unknown:
                    currentStage = stage._00_preStart;
                    break;
                case _00_preStart:
                    currentStage = stage._10_Drive_Out;
                    break;

                case _10_Drive_Out:
                    follower.followPath(scorePreload);
                    currentStage = stage._20_LaunchPreload;
                    break;
                case _20_LaunchPreload:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the launchPose's position */
                    if (!follower.isBusy()) {
                        // call the launcher command. Then set the stage to the next step.
                        // In the next step, check if this step is complete before executing that next step.
                        robot.launcher.cmdOutnear();

                        currentStage=stage._120_End;

                    }
                    break;

                case _120_End:
                    // if previous command complete stop everything
                    robot.stop();
            } //end of switch

                follower.update();

                telemetryMU.addData("Update ", thisUpdate);
                // Feedback to Driver Hub for debugging
                telemetryMU.addData("current stage", currentStage);
                telemetryMU.addData("x", follower.getPose().getX());
                telemetryMU.addData("y", follower.getPose().getY());
                telemetryMU.addData("heading Degrees:", Math.toDegrees(follower.getPose().getHeading()));
                //    telemetryMU.addData("path", follower.getCurrentPath());
                telemetryMU.addData("y", follower.getPose().getY());
                telemetryMU.addData("heading", follower.getPose().getHeading());

                telemetryMU.update();
                //org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawDebug(follower);


        }

        /**
         * This method is called once at the init of the OpMode.
         **/
        @Override
        public void init() {
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            msStuckDetectInit = Settings.msStuckDetectInit;
            msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
            msStuckDetectStart = Settings.msStuckDetectStart;
            msStuckDetectLoop = Settings.msStuckDetectLoop;
            msStuckDetectStop = Settings.msStuckDetectStop;

            robot.hardwareMap = hardwareMap;
            robot.telemetry = telemetry;
            robot.init();
            telemetry.addData("Red Near Auton", "Initialized");


            follower = CompBotConstants.createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);
            follower.update();
            //  display info in pedro panels.
            //org.firstinspires.ftc.teamcode.pedroPathing.Drawing.init();
            telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

            // disp[lay starting postition
            telemetryMU.addData("initialized postition - Update ", thisUpdate);
            // Feedback to Driver Hub for debugging
            telemetryMU.addData("Current stage", currentStage);
            telemetryMU.addData("x", follower.getPose().getX());
            telemetryMU.addData("y", follower.getPose().getY());
            telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
            //  telemetryMU.addData("path", follower.getCurrentPath());
            telemetryMU.addData("y", follower.getPose().getY());
            telemetryMU.addData("heading", follower.getPose().getHeading());

            telemetryMU.update();
       //     Drawi.drawDebug(follower);


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

        }

        /**
         * We do not use this because everything should automatically disable
         **/
        @Override
        public void stop() {
            robot.stop();
        }


    /**
     * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
     *
     * @author Lazar - 19234
     * @version 1.1, 5/19/2025
//     */
//    class Drawing {
//        public static final double ROBOT_RADIUS = 9; // woah
//        private final FieldManager panelsField = PanelsField.INSTANCE.getField();
//
//        private final Style robotLook = new Style(
//                "", "#3F51B5", 0.75
//        );
//        private final Style historyLook = new Style(
//                "", "#4CAF50", 0.75
//        );
//
//        /**
//         * This prepares Panels Field for using Pedro Offsets
//         */
//        public void init() {
//            panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
//        }
//
//        /**
//         * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
//         * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
//         *
//         * @param follower Pedro Follower instance.
//         */
//        public void drawDebug(Follower follower) {
//            if (follower.getCurrentPathChain() != null && follower.getCurrentPath() != null) {
//                drawPath(follower.getCurrentPathChain(), robotLook);
//                Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//                drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
//            } else if (follower.getCurrentPath() != null) {
//                drawPath(follower.getCurrentPath(), robotLook);
//                Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
//                drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
//            }
//
//            drawPoseHistory(follower.getPoseHistory(), historyLook);
//            drawRobot(follower.getPose(), historyLook);
//
//            sendPacket();
//        }
//
//        /**
//         * This draws a robot at a specified Pose with a specified
//         * look. The heading is represented as a line.
//         *
//         * @param pose  the Pose to draw the robot at
//         * @param style the parameters used to draw the robot with
//         */
//        public void drawRobot(Pose pose, Style style) {
//            if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
//                return;
//            }
//
//            panelsField.setStyle(style);
//            panelsField.moveCursor(pose.getX(), pose.getY());
//            panelsField.circle(ROBOT_RADIUS);
//
//            Vector v = pose.getHeadingAsUnitVector();
//            v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
//            double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
//            double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();
//
//            panelsField.setStyle(style);
//            panelsField.moveCursor(x1, y1);
//            panelsField.line(x2, y2);
//        }
//
//        /**
//         * This draws a robot at a specified Pose. The heading is represented as a line.
//         *
//         * @param pose the Pose to draw the robot at
//         */
//        public void drawRobot(Pose pose) {
//            drawRobot(pose, robotLook);
//        }
//
//        /**
//         * This draws a Path with a specified look.
//         *
//         * @param path  the Path to draw
//         * @param style the parameters used to draw the Path with
//         */
//        public void drawPath(Path path, Style style) {
//            double[][] points = path.getPanelsDrawingPoints();
//
//            for (int i = 0; i < points[0].length; i++) {
//                for (int j = 0; j < points.length; j++) {
//                    if (Double.isNaN(points[j][i])) {
//                        points[j][i] = 0;
//                    }
//                }
//            }
//
//            panelsField.setStyle(style);
//            for (int i = 0; i < points[0].length - 1; i++) {
//                panelsField.moveCursor(points[0][i], points[1][i]);
//                panelsField.line(points[0][i + 1], points[1][i + 1]);
//            }
//        }
//
//        /**
//         * This draws all the Paths in a PathChain with a
//         * specified look.
//         *
//         * @param pathChain the PathChain to draw
//         * @param style     the parameters used to draw the PathChain with
//         */
//        public void drawPath(PathChain pathChain, Style style) {
//            for (int i = 0; i < pathChain.size(); i++) {
//                drawPath(pathChain.getPath(i), style);
//            }
//        }
//
//        /**
//         * This draws the pose history of the robot.
//         *
//         * @param poseTracker the PoseHistory to get the pose history from
//         * @param style       the parameters used to draw the pose history with
//         */
//        public void drawPoseHistory(PoseHistory poseTracker, Style style) {
//            panelsField.setStyle(style);
//
//            int size = poseTracker.getXPositionsArray().length;
//            for (int i = 0; i < size - 1; i++) {
//
//                panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
//                panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
//            }
//        }
//
//        /**
//         * This draws the pose history of the robot.
//         *
//         * @param poseTracker the PoseHistory to get the pose history from
//         */
//        public void drawPoseHistory(PoseHistory poseTracker) {
//            drawPoseHistory(poseTracker, historyLook);
//        }
//
//        /**
//         * This tries to send the current packet to FTControl Panels.
//         */
//        public void sendPacket() {
//            panelsField.update();
//        }
//
//    }

    }
