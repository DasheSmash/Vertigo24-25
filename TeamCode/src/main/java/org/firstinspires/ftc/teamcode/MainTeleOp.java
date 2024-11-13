//Allows for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Imports necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    //Code that will run when the user presses 'INIT':
    @Override
    public void runOpMode() {
        //Variable declaration, initialization, and instantiation:
        ElapsedTime runtime = new ElapsedTime();
        RobotMethods RMO = new RobotMethods(hardwareMap);
        //Sets the robot's wheels to go 'backwards', and for the robot's motors to brake when at 0 power:
        RMO.SetDirectionBackwards();
        RMO.setZeroBehaviorAll();

        //Variables to move the robot:
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        //Variables for drive speed, arm speed, manual arm movement, and preset arm positions:
        double driveMultiplier = 0.5;
        double armMultiplier = 1;
        double distanceMultiplier = 1;
        double armChange = 0.0;
        double armDistance = 0.0;
        int estArmPos = 0;
        int estSlidePos = 0;
        int changePos = -1;
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}}; // Preset arm positions

        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {

            // Pressing "back" on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}

            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);

            //Determines speed setting:
            if (gamepad1.right_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.left_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}

            if (gamepad2.right_trigger > 0.1f){distanceMultiplier = 8;}
            else if (gamepad2.left_trigger > 0.1f){distanceMultiplier = 4;}
            else{distanceMultiplier = 6;}

            /*if (gamepad2.left_bumper){armMultiplier = 2;}
            else if (gamepad2.right_bumper){armMultiplier = 0.5;}
            else{armMultiplier = 1;}*/

            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,yaw);

            //Preset arm position code:
            if (gamepad2.dpad_left){changePos = -1;}
            //Full Back (To starting position):
            else if (gamepad2.a) {
                changePos = 0;
            }
            //Full forward to intake pixels:
            else if (gamepad2.b) {
                changePos = 1;
            }
            //Deposit pixels on Backboard:
            else if (gamepad2.x) {
                changePos = 2;
            }
            //Deposit pixels on stripe:
            else if (gamepad2.y) {
                changePos = 3;
            }
            //if(gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.y){armChange = 0;}
            //if(changePos != -1){RMO.setArmDegree((int)(armPositions[changePos][0] + armChange));}

            //Arm Code:
            //DO: make estArmPos unable to go above 15 degrees, and unable to go below -80 degrees
            armChange = gamepad2.right_stick_y * armMultiplier;
            armDistance = gamepad2.left_stick_y * distanceMultiplier;
            if (Math.abs(armChange) > 0.1){
                estArmPos += (int)armChange;
                RMO.setArmDegree(estArmPos);
            }
            if (Math.abs(armDistance) > 0.1){
                estSlidePos += (int)armDistance;
                estSlidePos = Math.max(estSlidePos, -2700);
                estSlidePos = Math.min(estSlidePos, 0);
                RMO.setArmDistance(estSlidePos);
            }
            if(gamepad2.right_bumper){RMO.openClaw(0.05);}
            else if(gamepad2.left_bumper){RMO.closeClaw(0.05);}
            //else{RMO.stopClaw();}

            //Sends data back to the driver's station:
            //telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("arm Position:", RMO.armMotor.getCurrentPosition());
            telemetry.addData("Gear position: ", RMO.armMotor.getCurrentPosition()/(3*1.5));
            telemetry.addData("Position: ", changePos);
            telemetry.addData("G2_RS_Y: ", gamepad2.right_stick_y);
            telemetry.addData("G2_LS_Y: ", gamepad2.left_stick_y);
            telemetry.addData("Arm Target: ", estArmPos);
            telemetry.addData("Slide Target: ", estSlidePos);
            telemetry.addData("Linslide current detected pos: ", RMO.linearMotor.getCurrentPosition()); //Max ~4000 degrees
            telemetry.addData("Linslide current actual pos: ", RMO.getSlideDegree()); //Max ~2650 degrees
            telemetry.addData("Claw1 pos: ", RMO.claw1.getPosition());
            telemetry.update();
        }
    }
}