/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TechiesAuto:WithCamera", group = "Concept")
// @Disabled
public class TechiesAutoWithCameraDection extends LinearOpMode {


    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double DRIVE_SPEED_3 = 0.3;
    static final double DRIVE_SPEED_6 = 0.6;
    static final double DRIVE_SPEED_2 = 0.2;


    static final double TURN_SPEED = 0.5;

    public static final int TARGET_ZONE_A = 0;
    public static final int TARGET_ZONE_B = 1;
    public static final int TARGET_ZONE_C = 4;
    public static final int TARGET_ZONE_DEFAULT = TARGET_ZONE_A;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private ElapsedTime runtime = new ElapsedTime();
    TechiesHardwareBot robot = new TechiesHardwareBot();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    private static final String VUFORIA_KEY =
            " AbfemMf/////AAABmVQ5LwVH3Umfv+Oiv7oNSvcdOBa+ogwEc69mgH/qFNgbn1NXBJsX4J5R6N4SYojjxFB6eHjdTHaT3i9ZymELzgaFrPziL5B/TX2/dkxnIK5dcOjLHOZu2K3jVPYciJnwj20ZmRXSN46Y4uMdzWSJ3X1wgKovNQzZvx+7dljIonRJfLjSF5aSuoTDqEkdcsGTJ92J7jgc5jN53Vml3rAI+qPUTT8qpI8T1enV6NYublcUMpofpovmHsH9kvI+U1h9Rc8cXwbGPlr3PVoKQOwuZA0Y98Jywey6URYTswzpbMw8cWu4PMisB2ujpf0VEjiV6jjofr3OVRj6r5lEJZsnElY2mOhXdgVqJndvXYvCSpyI ";
    private VuforiaLocalizer vuforia;
    //{@link #tfod} is the variable we will use to store our instance of the TensorFlow Object Detection engine.
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initiateRobot();
        telemetry.addData(">", "runOpMode, init camera");
        telemetry.update();
        initVuforia();
        initTfod();
        activateCamera();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int targetZone = determineTargetZone();
        telemetry.addData("Target zone", targetZone);

        DropGoal(targetZone);
        driveToShooter(targetZone);
        accelerate();
        shoot();
        getwobblegoal(targetZone);
        //getring(targetZone);
        //driveToShooter2(targetZone);
        //accelerate();
        //shoot();
        //driveToLine(targetZone);
        shutDownCamera();
    }

    protected void initiateRobot() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        setEncoder(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoder(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftBack.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        telemetry.update();
    }

    protected void activateCamera() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            telemetry.addData(">", "activate camera");
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //  tfod.setZoom(2.5, 16/9.0);
        }
    }

    protected int determineTargetZone() {
        int targetZone = TARGET_ZONE_DEFAULT;
        if (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    //KL test
                    if (updatedRecognitions.size() == 0) {
                        // empty list. no objects recognized
                        telemetry.addData("TF0D", "No item detected");
                        telemetry.addData("Target Zone", "A");
                        targetZone = TARGET_ZONE_A;
                    } else {
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                targetZone = TARGET_ZONE_B;
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                targetZone = TARGET_ZONE_C;
                            }
                        }
                    }
                    telemetry.update();
                    //       }
                } // endif tfod!=null
            }
        }

        return targetZone;
    }


    protected void shutDownCamera() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = .5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void setEncoder(DcMotor.RunMode stopAndResetEncoder) {
        robot.leftDrive.setMode(stopAndResetEncoder);
        robot.rightDrive.setMode(stopAndResetEncoder);
        robot.leftBack.setMode(stopAndResetEncoder);
        robot.rightBack.setMode(stopAndResetEncoder);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            setEncoder(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                            robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget,
                        newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            setEncoder(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    private void DropGoal(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            DriveToAWithEncode();

        } else if (TARGET_ZONE_B == targetZone) {
            encoderDrive(DRIVE_SPEED, -51, -51, -51, -51, 5.0);
            encoderDrive(DRIVE_SPEED, 0, -12, 0, -12, 5.0);
            encoderDrive(DRIVE_SPEED, -60, -60, -60, -60, 5.0);

        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
            encoderDrive(DRIVE_SPEED, -2.5, 2.5, -2.5, 2.5, 5.0);
            encoderDrive(DRIVE_SPEED, -142, -142, -142, -142, 5.0);

        }

    }

    private void DriveToAWithEncode() {
        double i = 3;
        int i2 = 72;
        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
        encoderDrive(DRIVE_SPEED, -i, i, -i, i, 5.0);
        encoderDrive(DRIVE_SPEED, -i2, -i2, -i2, -i2, 5.0);
    }


    private void driveToShooter(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5.0);
            encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 5.0);
            encoderDrive(DRIVE_SPEED, -29, 29, 29, -29, 5.0);
            encoderDrive(DRIVE_SPEED, -11, -11, -11, -11, 5.0);
        } else if (TARGET_ZONE_B == targetZone) {
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);
            encoderDrive(DRIVE_SPEED, -4, 4, -4, 4, 5.0);
            encoderDrive(DRIVE_SPEED, 18, 18, 18, 18, 5.0);
            encoderDrive(DRIVE_SPEED, -9, 9, 9, -9, 5.0);
        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);
            encoderDrive(DRIVE_SPEED, 3.3, -3.3, 3.3, -3.3, 5.0);
            encoderDrive(DRIVE_SPEED, 51, 51, 51, 51, 5.0);
            encoderDrive(DRIVE_SPEED, -33, 33, 33, -33, 5.0);
        }
    }

    private void driveToLine(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
        } else if (TARGET_ZONE_B == targetZone) {
            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
        }
    }

    private void accelerate() {

        DcMotor shooter1 = robot.getShooter1();
        DcMotor shooter2 = robot.getShooter2();
        shooter1.setPower(.37);
        shooter2.setPower(.37);
        sleep(3000);

    }


    private void shoot() {
        DcMotor shooter1 = robot.getShooter1();
        DcMotor shooter2 = robot.getShooter2();
        DcMotor intake = robot.getIntake();
        shooter1.setPower(.39);
        shooter2.setPower(.39);
        intake.setPower(.33);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        intake.setPower(0);
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    private void getring(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            sleep(1);
        } else if (TARGET_ZONE_B == targetZone) {
            DcMotor shooter1 = robot.getShooter1();
            DcMotor shooter2 = robot.getShooter2();
            shooter1.setPower(-.13);
            shooter2.setPower(-.13);
            DcMotor intake = robot.getIntake();
            intake.setPower(.6);
            while (opModeIsActive() && (runtime.seconds() < 5.0)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            encoderDrive(DRIVE_SPEED_3, 30, 30, 30, 30, 5.0);

            intake.setPower(0);
        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, 25, 25, 25, 25, 5.0);
            encoderDrive(DRIVE_SPEED_6, -10, -10, -10, -10, 5.0);
            DcMotor intake = robot.getIntake();
            DcMotor shooter1 = robot.getShooter1();
            DcMotor shooter2 = robot.getShooter2();
            shooter1.setPower(-.13);
            shooter2.setPower(-.13);
            intake.setPower(.7);
            encoderDrive(DRIVE_SPEED_2, 23, 23, 23, 23, 5.0);
            while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            intake.setPower(-.5);
            while (opModeIsActive() && (runtime.seconds() < .5)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            intake.setPower(0);
        }
    }

    private void driveToShooter2(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            sleep(1);
        } else if (TARGET_ZONE_B == targetZone) {
            encoderDrive(DRIVE_SPEED, -28, -28, -28, -28, 5.0);
        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, -45, -45, -45, -45, 5.0);
        }
    }
    private void getwobblegoal(int targetZone) {
        if (TARGET_ZONE_A == targetZone) {
            encoderDrive(DRIVE_SPEED, 58, -58, 58, -58, 5.0);
            encoderDrive(DRIVE_SPEED, -50, -50, -50, -50, 5.0);

            Servo Grabber = robot.getGrabber();
            DcMotor arm = robot.getArm();
            arm.setPower(.4);
            while (opModeIsActive() && (runtime.seconds() < 1.6)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            sleep(2500);
            encoderDrive(DRIVE_SPEED, -14, 14, -14, 14, 5.0);

            Grabber.setPosition(.35);
            encoderDrive(DRIVE_SPEED, 65, -65, 65, -65, 5.0);
            encoderDrive(DRIVE_SPEED, -53, -53, -53, -53, 5.0);
            Grabber.setPosition(0);
            arm.setPower(-.4);
            while (opModeIsActive() && (runtime.seconds() < 1.6)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);
            encoderDrive(DRIVE_SPEED, 25, -25, 25, -25, 5.0);
            encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5.0);


        } else if (TARGET_ZONE_B == targetZone) {
            encoderDrive(DRIVE_SPEED, 58, -58, 58, -58, 5.0);
            encoderDrive(DRIVE_SPEED, -50, -50, -50, -50, 5.0);

            Servo Grabber = robot.getGrabber();
            DcMotor arm = robot.getArm();
            arm.setPower(.4);
            while (opModeIsActive() && (runtime.seconds() < 1.6)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            sleep(2500);
            encoderDrive(DRIVE_SPEED, -14, 14, -14, 14, 5.0);

            Grabber.setPosition(.35);
            sleep(1000);

            encoderDrive(DRIVE_SPEED, -46, 46, -46, 46, 5.0);
            encoderDrive(DRIVE_SPEED, -72, -72, -72, -72, 5.0);
            Grabber.setPosition(0);
            sleep(500);
            arm.setPower(-.8);
            while (opModeIsActive() && (runtime.seconds() < 1.6)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        } else if (TARGET_ZONE_C == targetZone) {
            encoderDrive(DRIVE_SPEED, 58, -58, 58, -58, 5.0);
            encoderDrive(DRIVE_SPEED, -50, -50, -50, -50, 5.0);

            Servo Grabber = robot.getGrabber();
            DcMotor arm = robot.getArm();
            arm.setPower(.8);
            while (opModeIsActive() && (runtime.seconds() < .9)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            sleep(1100);
            encoderDrive(DRIVE_SPEED, -14, 14, -14, 14, 5.0);

            Grabber.setPosition(.35);
            sleep(700);
            encoderDrive(DRIVE_SPEED, 25, 25, 25, 25, 5.0);
            encoderDrive(DRIVE_SPEED, -53, 53, -53, 53, 5.0);
            encoderDrive(DRIVE_SPEED, -110, -110, -110, -110, 5.0);
            Grabber.setPosition(0);
            sleep(200);
            arm.setPower(-.8);
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            encoderDrive(DRIVE_SPEED, 45, 45, 45, 45, 5.0);

        }
    }
}

