#include "main.h"
#include <cmath>

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors

// Left side: Port 1 reversed, Port 2 reversed
pros::MotorGroup left_motors({-1, -12}, pros::v5::MotorGears::blue); 

// Right side: Port 10, Port 12
pros::MotorGroup right_motors({10, 14}, pros::v5::MotorGears::blue);

// TOGGLE motor
pros::Motor toggle_motor(6, pros::v5::MotorGears::green);
// pros::Motor intake_motor(6, pros::v5::MotorGears::green);

// Pneumatics -> | A - PORT, False - default state (retracted)
pros::adi::Pneumatics clamp('A', false);
// Elevator motors - ADJUST ports and gearset

// Full Motors Group
pros::MotorGroup full_motors({17, 20}, pros::v5::MotorGears::green);

// Half Motors Group 
pros::MotorGroup half_motors({18, -19}, pros::v5::MotorGears::green);

// IMU
// pros::Imu imu(0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// all null for now without tracking wheels/imu
// see tracking wheels section -> https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html#odometry
lemlib::OdomSensors sensors(
    nullptr, nullptr, nullptr, nullptr, nullptr
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	
	// calibration is commented since no imu right now
    // chassis.calibrate(); // calibrate sensors

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "Elev Pos: %.1f deg", full_motors.get_position()); // print elevator position
            // delay to save resources
            pros::delay(20);
        }
    });

    // Set brake mode for motor groups
    full_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    half_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Set brake mode for toggle
    toggle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Set elevator encoders to degrees
    full_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    // ASSUMES THAT WHEN ROBOT IS TURNED ON, ELEVATOR IS AT BOTTOM (0 degrees)
    full_motors.tare_position();

}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

void toggleSkills() {
    // Drive forward
    left_motors.move(30);
    right_motors.move(30);
    pros::delay(100);
    // Drive back into toggle
    left_motors.move(-30);
    right_motors.move(-30);
    pros::delay(100);
    // Drive forward 
    left_motors.move(30);
    right_motors.move(30);
    pros::delay(100);
    // Drive back into toggle
    left_motors.move(-30);
    right_motors.move(-30);
    pros::delay(100);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    toggleSkills();
}
// UNTUNED -> CHECK LATER
// 360 degrees -> One motor rotation
const double levelPositions[4] = {0, 360, 720};

void moveElevator(int destination) {
    // Move full_motors to the level w/ 100 RPM
    full_motors.move_absolute(levelPositions[destination], 100);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	int currentLevel = 0; // Where the elevator is right now (0 to 2)
    int targetLevel = 0; // The level to go to (0 to 2)
    const int minLevel = 0; // Min level
    const int maxLevel = 2; // Max level

    bool isMoving = false;
    int moveTimer = 0;

    bool armrunningu=false;
    bool armrunningd=false; 
    bool elevator_runu=false; 
    bool elevator_rund=false;
    bool piston_extended=false;

	while (true) {
        // Left stick UP/DOWN -> throttle 
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // Right stick LEFT/RIGHT -> turn 
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // arcade drive
        chassis.arcade(throttle, turn);

        // PNEUMATICS CONTROL -> Y Toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            pros::lcd::print(4, "y pressed: %d", targetLevel);
            piston_extended = !piston_extended; // toggle the piston state
            if (piston_extended==false) {
                    clamp.extend(); // extend the piston
            } else {
                    clamp.retract(); // retract the piston
            }
        }

        // Toggle control -> B Toggle
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            pros::lcd::print(4, "b pressed: %d", targetLevel);
            toggle_motor.move(40); // Move toggle motor
        } else {
            toggle_motor.brake(); // Stop toggle motor
        }

        // --- ELEVATOR LEVEL CONTROL ---
        // L1 = UP, L2 = DOWN
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (targetLevel < maxLevel) {
                targetLevel++;
                pros::lcd::print(4, "l1 pressed: %d", targetLevel);
                pros::lcd::print(3, "Selected Level: %d", targetLevel);
            }
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (targetLevel > minLevel) {
                targetLevel--;
                pros::lcd::print(4, "l2 pressed: %d", targetLevel);
                pros::lcd::print(3, "Selected Level: %d", targetLevel);
            }
        }

        // Start movement (Button A)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (targetLevel != currentLevel) {
                pros::lcd::print(4, "a pressed: %d", targetLevel);
                full_motors.move_absolute(levelPositions[targetLevel], 100);
                isMoving = true;
                moveTimer = 0; // Reset timer
                pros::lcd::print(3, "Target Level: %d", targetLevel);
            }
        }

        // BACKUP ELEVATOR CONTROL -> UP (L1) / DOWN (L2)
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        //     full_motors.brake();
        // }
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        //     elevator_runu = !elevator_runu;
        //     elevator_rund = false;
        // }
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
        //     elevator_rund = !elevator_rund;
        //     elevator_runu = false;
        // }
        // if (elevator_runu == true){
        //     full_motors.move(127);
        // } else if (elevator_rund == true){
        //     full_motors.move(-127);
        // } else {
        //     full_motors.brake();
        // }

        // Track motor
        if (isMoving) {
            double error = levelPositions[targetLevel] - full_motors.get_position();
            moveTimer += 10; // Increse timer by 10ms to match delay at bottom of opcontrol loop

            // Check if within 10 degrees of target
            if (fabs(error) < 10.0) {
                full_motors.brake();
                currentLevel = targetLevel; // Update current level
                isMoving = false;
                pros::lcd::print(3, "Level: %d", currentLevel);
            } 

            // Timeout after 5 seconds
            else if (moveTimer >= 5000) {
                full_motors.brake();
                isMoving = false;
                pros::lcd::print(3, "elevator timeout");
            }
        }
       
        // HALF MOTORS CONTROL -> UP (UP BUTTON) / DOWN (DOWN BUTTON)
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        //     armrunningu=!armrunningu;
        //     armrunningd=false;
        // }
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //     armrunningd=!armrunningd;
        //     armrunningu=false;
        // }
        // if (armrunningu == true){
        //     half_motors.move(40);
        // } else if(armrunningd == true) {
        //     half_motors.move(-40);
        // } else {
        //     half_motors.brake();
        // }

        // delay to save resources
        pros::delay(10);
    }
}