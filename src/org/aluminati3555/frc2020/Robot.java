/**
 * Copyright (c) 2020 Team 3555
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.aluminati3555.frc2020;

import org.aluminati3555.lib.auto.AluminatiAutoSelector;
import org.aluminati3555.lib.auto.AluminatiAutoTask;
import org.aluminati3555.lib.auto.AluminatiAutoSelector.Entry;
import org.aluminati3555.lib.data.AluminatiData;
import org.aluminati3555.lib.drivers.AluminatiMotorGroup;
import org.aluminati3555.lib.drivers.AluminatiDisplay;
import org.aluminati3555.lib.drivers.AluminatiDisplay.Button;
import org.aluminati3555.lib.drivers.AluminatiDualGyro;
import org.aluminati3555.lib.drivers.AluminatiJoystick;
import org.aluminati3555.lib.drivers.AluminatiLEDDriver;
import org.aluminati3555.lib.drivers.AluminatiLEDDriver.Mode;
import org.aluminati3555.lib.drivers.AluminatiPigeon;
import org.aluminati3555.lib.loops.Loop;
import org.aluminati3555.lib.loops.Looper;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiVictorSPX;
import org.aluminati3555.lib.pneumatics.AluminatiCompressor;
import org.aluminati3555.lib.pneumatics.AluminatiDoubleSolenoid;
import org.aluminati3555.lib.robot.AluminatiRobot;
import org.aluminati3555.lib.trajectoryfollowingmotion.AluminatiRobotStateEstimator;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.util.AluminatiUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

import org.aluminati3555.frc2020.auto.ModeCharacterizeDrive;
import org.aluminati3555.frc2020.auto.ModeDoNothing;
import org.aluminati3555.frc2020.auto.ModeExamplePath;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.frc2020.systems.SpinnerSystem;

/**
 * This is the main class of the robot
 * 
 * @author Caleb Heydon
 */
public class Robot extends AluminatiRobot {
  // Robot state
  private RobotMode robotMode;
  private RobotState robotState;
  private AluminatiAutoTask autoTask;

  private boolean matchStarted;

  private ControlPanelColor controlPanelColor;

  // Looper
  private Looper looper;

  // Robot state estimator
  private AluminatiRobotStateEstimator robotStateEstimator;

  // Power distribution
  private PowerDistributionPanel pdp;

  // LED driver
  private AluminatiLEDDriver ledDriver;

  // Digit display
  private DisplayMode displayMode;
  private AluminatiDisplay display;

  // Joysticks
  private AluminatiJoystick driverJoystick;
  private AluminatiJoystick operatorJoystick;

  // Systems
  private DriveSystem driveSystem;
  private SpinnerSystem spinnerSystem;
  private ShooterSystem shooterSystem;

  // Pneumatics
  private AluminatiCompressor compressor;

  // Auto selector
  private AluminatiAutoSelector autoSelector;

  @Override
  public void robotInit() {
    // Configure pid
    AluminatiData.velocityKF = 0.3;
    AluminatiData.velocityKP = 0.2;
    AluminatiData.velocityKI = 0.0001;
    AluminatiData.velocityKD = 0.25;

    // Configure pure pursuit
    AluminatiData.pathFollowingProfileKP = 5;
    AluminatiData.pathFollowingProfileKI = 0.0001;
    AluminatiData.pathFollowingProfileKV = 0.007;
    AluminatiData.pathFollowingProfileKS = 0.076;

    AluminatiData.pathFollowingMaxVel = 140;
    AluminatiData.pathFollowingMaxAccel = 100;

    AluminatiUtil.generatePathFollowingFeedforwardValues();

    // Set encoder data
    AluminatiData.encoderUnitsPerRotation = 4096;

    // Set robot physical constants
    AluminatiData.wheelDiamater = 6;
    AluminatiData.driveWidth = 21;

    // Set thread priority
    Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
    Thread.currentThread().setName("Robot-Thread");

    // Set default robot state and mode
    robotState = new RobotState();
    robotMode = RobotMode.OPERATOR_CONTROL;

    // Make it clear that matchStarted is false
    matchStarted = false;

    // Set control panel color to unkown
    controlPanelColor = ControlPanelColor.UNKOWN;

    // Setup looper
    looper = new Looper();

    // Disable LiveWindow telemetry
    LiveWindow.disableAllTelemetry();

    // Setup pdp
    pdp = new PowerDistributionPanel();
    pdp.clearStickyFaults();

    // Setup led driver
    ledDriver = new AluminatiLEDDriver(0);

    // Setup digit display
    displayMode = DisplayMode.BATTERY_VOLTAGE;
    display = new AluminatiDisplay();

    // Setup joysticks
    driverJoystick = new AluminatiJoystick(0);
    operatorJoystick = new AluminatiJoystick(1);

    // Configure systems
    configureSystems();

    // Setup compressor
    compressor = new AluminatiCompressor();
    compressor.start();

    // Setup robot state estimator
    robotStateEstimator = new AluminatiRobotStateEstimator(robotState, driveSystem);
    looper.register(robotStateEstimator);

    // Setup data reporter
    looper.register(new DataReporter());

    // Start looper
    looper.start();

    // Setup auto selector
    autoSelector = new AluminatiAutoSelector(5810, new Entry("DoNothing", new ModeDoNothing()),
        new Entry("CharacterizeDrive", new ModeCharacterizeDrive(driveSystem)),
        new Entry("ExamplePath", new ModeExamplePath(robotState, driveSystem)),
        new Entry("Right6PowerCellTrench", null));
  }

  @Override
  public void robotPeriodic() {
    if (display.getButton(Button.BUTTON_A)) {
      displayMode = DisplayMode.BATTERY_VOLTAGE;
    } else if (display.getButton(Button.BUTTON_B)) {
      displayMode = DisplayMode.LOOP_TIME;
    }

    if (displayMode == DisplayMode.BATTERY_VOLTAGE) {
      display.display(pdp.getVoltage());
    } else {
      display.display(this.getLastDT() * 100);
    }
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    // Use brake mode if connected to the driverstation/fms
    if (DriverStation.getInstance().isDSAttached() || DriverStation.getInstance().isFMSAttached()) {
      driveSystem.brake();
    } else {
      // Use coast for vision calibration
      driveSystem.coast();
    }

    // Zero gyro if waiting for match to start
    if (!matchStarted) {
      driveSystem.getGyro().setHeading(Rotation2d.fromDegrees(0));
    }

    // Update systems
    double timestamp = Timer.getFPGATimestamp();
    driveSystem.update(timestamp, false);
    spinnerSystem.update(timestamp, false);
  }

  @Override
  public void autonomousInit() {
    matchStarted = true;

    // Set brake mode
    driveSystem.brake();

    // Stop auto task if one is running
    if (autoTask != null) {
      autoTask.stop();
    }

    double timestamp = Timer.getFPGATimestamp();

    // Reset robot state
    robotState.reset(timestamp, new Pose2d(0, 0, Rotation2d.fromDegrees(0)), driveSystem);

    loadAutoMode();
    if (autoTask != null) {
      robotMode = RobotMode.AUTONOMOUS;
      autoTask.start(timestamp);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Set brake mode
    driveSystem.brake();

    // Update leds
    updateLEDS(false, true);

    double timestamp = Timer.getFPGATimestamp();

    autoControl(timestamp);

    // Update systems
    driveSystem.update(timestamp, false);
    spinnerSystem.update(timestamp, false);
  }

  @Override
  public void teleopInit() {
    matchStarted = true;

    // Set brake mode
    driveSystem.brake();
  }

  @Override
  public void teleopPeriodic() {
    boolean enabled = (robotMode == RobotMode.OPERATOR_CONTROL);

    // Set brake mode
    driveSystem.brake();

    // Update leds
    updateLEDS(enabled, false);

    // Fetch control panel color if we do not already have it
    if (controlPanelColor == ControlPanelColor.UNKOWN) {
      fetchControlPanelColor();
    }

    double timestamp = Timer.getFPGATimestamp();

    autoControl(timestamp);

    // Update systems
    driveSystem.update(timestamp, enabled);
    spinnerSystem.update(timestamp, enabled);
  }

  @Override
  public void testInit() {
    // Set coast mode
    driveSystem.coast();
  }

  @Override
  public void testPeriodic() {
    // Set coast mode
    driveSystem.coast();

    updateLEDS(true, false);
  }

  /**
   * Configures the robot systems
   */
  private void configureSystems() {
    // Setup drivetrain
    AluminatiMotorGroup left = new AluminatiMotorGroup(new AluminatiTalonSRX(40), new AluminatiTalonSRX(41),
        new AluminatiVictorSPX(42));
    AluminatiMotorGroup right = new AluminatiMotorGroup(true, new AluminatiTalonSRX(50), new AluminatiTalonSRX(51),
        new AluminatiVictorSPX(52));

    AluminatiPigeon gyro1 = new AluminatiPigeon((AluminatiTalonSRX) left.getMotors()[1]);
    AluminatiPigeon gyro2 = new AluminatiPigeon((AluminatiTalonSRX) right.getMotors()[1]);
    AluminatiDualGyro dualGyro = new AluminatiDualGyro(gyro1, gyro2);

    left.getMaster().setSensorPhase(true);
    right.getMaster().setSensorPhase(true);
    driveSystem = new DriveSystem(looper, robotState, left, right, dualGyro, driverJoystick);

    spinnerSystem = new SpinnerSystem(new AluminatiTalonSRX(60), new AluminatiDoubleSolenoid(0, 1));
    shooterSystem = new ShooterSystem(new AluminatiTalonSRX(70), new AluminatiDoubleSolenoid(2, 3));
  }

  /**
   * Loads the selected auto into autoTask
   */
  private void loadAutoMode() {
    autoTask = autoSelector.getSelected();
    if (autoTask == null) {
      autoTask = new ModeDoNothing();
    }
  }

  /**
   * Updates the LED driver
   */
  private void updateLEDS(boolean operatorControl, boolean auto) {
    if (!operatorControl && auto) {
      ledDriver.setMode(Mode.BLUE);
    } else if (operatorControl && !auto) {
      ledDriver.setMode(Mode.VIOLET);
    } else if (!operatorControl && !auto) {
      ledDriver.setMode(Mode.SCANNER);
    } else {
      ledDriver.setMode(Mode.OFF);
    }
  }

  /**
   * Controls the robot during auto
   */
  private void autoControl(double timestamp) {
    if (driverJoystick.getRawButtonPressed(11)) {
      // Stop task and cleanup
      autoTask.stop();
      robotMode = RobotMode.OPERATOR_CONTROL;
    }

    if (robotMode == RobotMode.AUTONOMOUS && autoTask != null) {
      if (autoTask.isComplete()) {
        // Stop task and cleanup
        autoTask.stop();
        robotMode = RobotMode.OPERATOR_CONTROL;
      } else {
        autoTask.update(timestamp);
      }
    }
  }

  /**
   * Fetches the control panel target color
   */
  private void fetchControlPanelColor() {
    String dataString = DriverStation.getInstance().getGameSpecificMessage();

    // Stop if the string is empty or null
    if (dataString == null || dataString.length() < 1) {
      return;
    }

    char color = dataString.charAt(0);

    // Synchronized to make this thread-safe when accessing from vision thread
    synchronized (controlPanelColor) {
      switch (color) {
      case 'B':
        controlPanelColor = ControlPanelColor.BLUE;
        break;
      case 'G':
        controlPanelColor = ControlPanelColor.GREEN;
        break;
      case 'R':
        controlPanelColor = ControlPanelColor.RED;
        break;
      case 'Y':
        controlPanelColor = ControlPanelColor.YELLOW;
        break;
      default:
        controlPanelColor = ControlPanelColor.UNKOWN;
        break;
      }
    }
  }

  private enum RobotMode {
    AUTONOMOUS, OPERATOR_CONTROL
  }

  private enum DisplayMode {
    BATTERY_VOLTAGE, LOOP_TIME
  }

  private enum ControlPanelColor {
    UNKOWN, BLUE, GREEN, RED, YELLOW
  }

  private class DataReporter implements Loop {
    public void onStart(double timestamp) {

    }

    public void onLoop(double timestamp) {
      // Do not report data if connected to the fms
      if (!DriverStation.getInstance().isFMSAttached()) {
        SmartDashboard.putNumber("leftPower", driveSystem.getLeftGroup().getMaster().getMotorOutputPercent());
        SmartDashboard.putNumber("rightPower", driveSystem.getRightGroup().getMaster().getMotorOutputPercent());
      }
    }

    public void onStop(double timestamp) {

    }

    public String getName() {
      return "[DataReporter]";
    }
  }
}
