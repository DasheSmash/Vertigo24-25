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
        double armMultiplier = 0.5;
        double armChange = 0.0;
        int changePos = -1;
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}}; // Preset arm positions
        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            // Pressing "back" on either of the controllers will stop the code
            // It's recommended that this is turned off during competition:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}
            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            //Determines speed setting:
            if (gamepad1.left_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.right_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}
            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,yaw);
            if (gamepad2.dpad_left){changePos = -1;}
            //Full Back (To starting position):
            else if (gamepad2.a) {
                changePos = 0;
            }
            //Full forward to intake pixels:
            else if (gamepad2.b || gamepad2.dpad_down) {
                changePos = 1;
            }
            //Deposit pixels on Backboard:
            else if (gamepad2.x || gamepad2.dpad_up) {
                changePos = 2;
            }
            //Deposit pixels on stripe:
            else if (gamepad2.y) {
                changePos = 3;
            }
            if(gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.y){armChange = 0;}
            /**/if (gamepad2.right_bumper){RMO.intakeSystem.setPosition(0.9);}
            /**/else if (gamepad2.left_bumper){RMO.intakeSystem.setPosition(0.1);}
            /**/else{RMO.intakeSystem.setPosition(0.5);}
            armChange += gamepad2.left_stick_y * armMultiplier;
            if(changePos != -1){RMO.setArmDegree((int)(armPositions[changePos][0] + armChange));/**/RMO.intakeJoint.setPosition(armPositions[changePos][1]);}
            else{RMO.armMotor.setPower(0);}
            /**/if(gamepad1.dpad_up){RMO.changeArmDegree(120);}
            /**/else if(gamepad1.dpad_down){RMO.changeArmDegree(-45);}
            //RMO.setArmDegreeChange((int)gamepad2.left_stick_y);
            //RMO.armMotor.setPower(gamepad2.left_stick_y);
            //Sends data back to the driver's station:
            telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("arm Position:", RMO.armMotor.getCurrentPosition());
            telemetry.addData("Gear position: ", RMO.armMotor.getCurrentPosition()/(3*1.5));
            telemetry.addData("Position: ", changePos);
            telemetry.update();
        }
    }
}