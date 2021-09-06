/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name= "XXX - Techies Autonomous Drive", group="Pushbot")
//@Disabled
public class TechiesAutonomousDrive extends LinearOpMode {

    //KL start
    public static final int TARGET_ZONE_A = 0;
    public static final int TARGET_ZONE_B = 1;
    public static final int TARGET_ZONE_C = 4;
    public static final int TARGET_ZONE_DEFAULT = TARGET_ZONE_B;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            " AbfemMf/////AAABmVQ5LwVH3Umfv+Oiv7oNSvcdOBa+ogwEc69mgH/qFNgbn1NXBJsX4J5R6N4SYojjxFB6eHjdTHaT3i9ZymELzgaFrPziL5B/TX2/dkxnIK5dcOjLHOZu2K3jVPYciJnwj20ZmRXSN46Y4uMdzWSJ3X1wgKovNQzZvx+7dljIonRJfLjSF5aSuoTDqEkdcsGTJ92J7jgc5jN53Vml3rAI+qPUTT8qpI8T1enV6NYublcUMpofpovmHsH9kvI+U1h9Rc8cXwbGPlr3PVoKQOwuZA0Y98Jywey6URYTswzpbMw8cWu4PMisB2ujpf0VEjiV6jjofr3OVRj6r5lEJZsnElY2mOhXdgVqJndvXYvCSpyI ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    int targetZone = TARGET_ZONE_DEFAULT;
    //KL end
    TechiesHardwareBot robot = new TechiesHardwareBot();

    //Declare OpMode members.
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {
        // KL start
        telemetry.addData(">", "runOpMode, before camera");
        //telemetry.update();
        initVuforia();
        initTfod();
        // KL End

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
       // telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
       // telemetry.update();



/*  don't shut down yet
        if (tfod != null) {
            tfod.shutdown();
        }
*/
        setEncoder(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoder(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftBack.getCurrentPosition(),
                robot.rightBack.getCurrentPosition());
        //telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        targetZone = determineTargetZone();
        telemetry.addData("Target zone", targetZone);    //
       // telemetry.update();


      //  DropGoal(targetZone);

        driveToShooter();
        accelerate();
        shoot();
        //intakering();
        //pickupring();
        //driveToShooter2();
        //sleep(2000);
        //accelerate();
        //shoot();
        driveToLine();




        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        //sleep(1000);     // pause for servos to move
       // shoot method call - Andrew 12/6
        telemetry.addData("Path", "Complete");
      //  telemetry.update();
    }

    protected int determineTargetZone() {
        int targetZoneAtStart = TARGET_ZONE_DEFAULT;
        if (opModeIsActive()) {
           // sleep(5000);
            while (opModeIsActive()) {
            //TODO:  let it loop a few times
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    //KL test
                    if(updatedRecognitions.size() == 0){
                        // empty list. no objects recognized
                        targetZone = TARGET_ZONE_A;
                        telemetry.addData("TF0D","No item detected");
                        telemetry.addData("Target Zone", "A");

                    }else {
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if (recognition.getLabel().equals("Single")) {
                                targetZone = TARGET_ZONE_B;
                                telemetry.addData("Target Zone", "B");
                            } else if (recognition.getLabel().equals("Quad")) {
                                targetZone = TARGET_ZONE_C;
                                telemetry.addData("Target Zone", "C");
                            }
                        }
                    }
                   // telemetry.update();
                }
            }
            } //closing for the while loop
        }
        return targetZoneAtStart;
    }

    //KL
    protected void DropGoal(int targetZone){
        if(TARGET_ZONE_A == targetZone){
            encoderDrive(DRIVE_SPEED, -85, -85, -85, -85,  5.0);
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
            encoderDrive(DRIVE_SPEED, -23, 23, 23, -23,  5.0);
            encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
            encoderDrive(DRIVE_SPEED, 27, -27, -27, 27,  5.0);
        }else if(TARGET_ZONE_B == targetZone){
            encoderDrive(DRIVE_SPEED, -117, -117, -117 , -117,  5.0);
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
            encoderDrive(DRIVE_SPEED, 20, -20, -20, 20,  5.0);
            encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
            encoderDrive(DRIVE_SPEED, -24, 24, 24, -24,  5.0);
        }else if(TARGET_ZONE_C == targetZone){
            encoderDrive(DRIVE_SPEED, -154, -154, -154, -154,  5.0);
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
            encoderDrive(DRIVE_SPEED, -23, 23, 23, -23,  5.0);
            encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
            encoderDrive(DRIVE_SPEED, 27, -27, -27, 27,  5.0);
        }

    }
     protected void driveToShooter(){

    }
    protected void driveToLine()
    {

    }
    public void accelerate() {

        //DcMotor shooter1 = robot.getShooter1();
        //DcMotor shooter2 = robot.getShooter2();
          //  shooter1.setPower(.38);
            //shooter2.setPower(.38);
            sleep(3000);

        }


    public void shoot() {
        //DcMotor shooter1 = robot.getShooter1();
        //DcMotor shooter2 = robot.getShooter2();
        //DcMotor intake = robot.getIntake();
        //shooter1.setPower(.4);
        //shooter2.setPower(.4);
        //intake.setPower(.37);
        sleep(5000);
        //intake.setPower(0);
    }

    private void setEncoder(DcMotor.RunMode stopAndResetEncoder) {
        robot.leftDrive.setMode(stopAndResetEncoder);
        robot.rightDrive.setMode(stopAndResetEncoder);
        robot.leftBack.setMode(stopAndResetEncoder);
        robot.rightBack.setMode(stopAndResetEncoder);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

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
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget,  newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                //
                //
                // telemetry.update();
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
}
