package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name= "XXX Techies Autonomous A", group="Pushbot")
public class AutoA extends TechiesAutonomous {


    protected void  DropGoal(){
        encoderDrive(DRIVE_SPEED, -85, -85, -85, -85,  5.0);
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
        encoderDrive(DRIVE_SPEED, -23, 23, 23, -23,  5.0);
        encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
        encoderDrive(DRIVE_SPEED, 27, -27, -27, 27,  5.0);
    }

    protected void driveToShooter(){
        encoderDrive(DRIVE_SPEED, -16, 16, 16, -16,  5.0);
        encoderDrive(DRIVE_SPEED, 19.2, 19.2, 19.2, 19.2,5.0);

        sleep(1000);
    }

    protected void driveToLine() {

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
    }


}
