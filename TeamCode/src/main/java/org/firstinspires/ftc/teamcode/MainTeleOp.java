//Allows for this file to use other files:
package org.firstinspires.ftc.teamcode;
//Imports necessary files:
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
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
        RMO.SetDirectionForward();
        RMO.setZeroBehaviorAll();
        RMO.startUsingDriveEncoders();

        //Variables to move the robot:
        double axial = 0.0;
        double lateral = 0.0;
        double yaw = 0.0;
        //Variables for drive speed, arm speed, manual arm movement, and preset arm positions:
        double driveMultiplier = 0.5;
        double armMultiplier = 15;
        double distanceMultiplier = 1;
        double slideGearRatio = 0.5;
        double climbMultiplier;
        double armChange = 0.0;
        double slideChange = 0.0;
        double armPos = 0.0;
        int slidePos = 0;
        int climbPos = 0;
        int changePos = -1;
        double[][] armPositions = {{0,0.98},{0,0.4},{170,0.95},{20,0.32}}; // (NOT IN USE) Preset arm positions
        boolean intake = false;
        boolean released = true;

        //Waits for the play button to be pressed:
        waitForStart();
        runtime.reset();
        //Repeatedly runs after the play button is pressed:
        while(opModeIsActive()) {
            //if(gamepad1.dpad_up){RMO.freeSlide();}
            // Pressing "back" on either of the controllers will stop the code:
            if (gamepad1.back || gamepad2.back){terminateOpModeNow();}

            //Changes where the robot will go according to the stick directions and the speed setting:
            axial = driveMultiplier * (-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            lateral = driveMultiplier * (gamepad1.left_stick_x);
            yaw = driveMultiplier * (-gamepad1.right_stick_x);
            if(Math.abs(gamepad2.left_trigger) > 0.1f){yaw += -gamepad2.right_stick_x * 0.25; lateral += gamepad2.left_stick_x * 0.25;}
            //if(Math.abs(gamepad2.left_stick_x) > 0.1f){lateral += gamepad2.left_stick_x * 0.25;}

            //Determines speed setting:
            if (gamepad1.right_trigger > 0.1f){driveMultiplier = 0.25;}
            else if (gamepad1.left_trigger > 0.1f){driveMultiplier = 1;}
            else{driveMultiplier = 0.5;}

            /*if (gamepad2.left_stick_button){distanceMultiplier = 80/slideGearRatio;}
            else*/ if (gamepad2.right_trigger > 0.1f){distanceMultiplier = 8/slideGearRatio;}
            else{distanceMultiplier = 24/slideGearRatio;}

            if(gamepad1.right_bumper){climbMultiplier = 8;}
            else if (gamepad1.left_bumper){climbMultiplier = 4;}
            else{climbMultiplier = 6;}

            /*if (gamepad2.left_bumper){armMultiplier = 2;}
            else if (gamepad2.right_bumper){armMultiplier = 0.5;}
            else{armMultiplier = 1;}*/

            //Implements the changes to the robot's position from other parts of the code:
            RMO.move(axial,lateral,-yaw);

            /*//Full Back (To starting position):
            if (gamepad2.a) {armPos = -600;}
            //Full forward to intake pixels:
            else if (gamepad2.b) {armPos = 450;}
            //Deposit pixels on Backboard:
            else if (gamepad2.x) {slidePos = 5;}
            //Deposit pixels on stripe:
            else if (gamepad2.y) {slidePos = -1750;}*/

            if(gamepad2.dpad_left){RMO.timedMotorMove(500, 0, -1, 0);} //Goes right
            else if (gamepad2.dpad_right){RMO.timedMotorMove(500, 0, 1, 0);} //Goes left
            if (gamepad2.dpad_up){RMO.timedMotorMove(500,0,0,1);} //Turns left
            else if (gamepad2.dpad_down){RMO.timedMotorMove(500,0,0,-1);} //Turns right
            //if(gamepad2.b || gamepad2.a || gamepad2.x || gamepad2.y){armChange = 0;}
            //if(changePos != -1){RMO.setArmDegree((int)(armPositions[changePos][0] + armChange));}

            //Arm Code:
            //DO: make armPos unable to go above 15 degrees, and unable to go below -80 degrees
            armChange = -gamepad2.right_stick_y * armMultiplier;
            slideChange = gamepad2.left_stick_y * distanceMultiplier;
            if (true){ //Condition temporarily disabled
                armPos += armChange;
                RMO.setArmDegree((int)armPos);
            }
            if (true){ //Condition temporarily disabled
                slidePos += (int)slideChange;
                slidePos = Math.max(slidePos, (int)(-3500/slideGearRatio));
                slidePos = Math.min(slidePos, 0);
                RMO.setArmDistance(slidePos);
            }
            /*
            if(gamepad2.right_bumper){RMO.openClaw();}
            else if(gamepad2.left_bumper){RMO.closeClaw();}
            else{RMO.stopClaw();}*/
            if(gamepad1.dpad_up){
                RMO.setDegreeForward(1080);
            }
            if(gamepad1.dpad_down){
                RMO.setDegreeForward(-1080);
            }
            if(gamepad1.dpad_left){
                RMO.setDegreeSide(-1080);
            }
            if(gamepad1.dpad_right){
                RMO.setDegreeSide(1080);
            }
            if(gamepad1.y){
                RMO.changeDegreeForward(1080);
            }
            if(gamepad1.a){
                RMO.changeDegreeForward(-1080);
            }
            if(gamepad1.x){
                RMO.changeDegreeSide(-1080);
            }
            if(gamepad1.b){
                RMO.changeDegreeSide(1080);
            }
            if(gamepad2.left_bumper && released){
                intake = !intake;
                released = false;
            }

            if (!gamepad2.left_bumper){released = true;}

            if(intake){
                if(gamepad2.right_bumper){
                    RMO.closeClaw();
                }else{
                    RMO.openClaw();
                }
            }

            /*if(gamepad1.dpad_up){RMO.startUsingDriveEncoders();}
            else if(gamepad1.dpad_down){RMO.stopUsingDriveEncoders();}*/

            //Sends data back to the driver's station:
            //telemetry.addData("Current centimeters from distance sensor: ", RMO.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("arm Position:", RMO.armMotor.getCurrentPosition());
            telemetry.addData("Gear position: ", RMO.armMotor.getCurrentPosition()/(3*1.5));
            telemetry.addData("Position: ", changePos);
            telemetry.addData("G2_RS_Y: ", gamepad2.right_stick_y);
            telemetry.addData("G2_LS_Y: ", gamepad2.left_stick_y);
            telemetry.addData("Arm Target: ", armPos);
            telemetry.addData("Slide Target: ", slidePos);
            telemetry.addData("Linslide current detected pos: ", RMO.linearMotor.getCurrentPosition()); //Max ~-1600 degrees
            telemetry.addData("Linslide current actual pos: ", RMO.getSlideDegree()); //Max ~2650 degrees
            telemetry.addData("Claw1 pos: ", RMO.claw.getPosition());
            telemetry.addData("FR pos: ", RMO.rightFrontDrive.getCurrentPosition());
            telemetry.addData("FL pos: ", RMO.leftFrontDrive.getCurrentPosition());
            telemetry.addData("BR pos: ", RMO.rightBackDrive.getCurrentPosition());
            telemetry.addData("BL pos: ", RMO.leftBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}