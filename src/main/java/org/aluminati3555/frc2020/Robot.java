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
import org.aluminati3555.lib.drivers.AluminatiPigeon;
import org.aluminati3555.lib.drivers.AluminatiSpark;
import org.aluminati3555.lib.loops.Loop;
import org.aluminati3555.lib.loops.Looper;
import org.aluminati3555.lib.drivers.AluminatiTalonSRX;
import org.aluminati3555.lib.drivers.AluminatiVictorSPX;
import org.aluminati3555.lib.drivers.AluminatiXboxController;
import org.aluminati3555.lib.pneumatics.AluminatiCompressor;
import org.aluminati3555.lib.pneumatics.AluminatiSolenoid;
import org.aluminati3555.lib.robot.AluminatiRobot;
import org.aluminati3555.lib.system.AluminatiSystem.SystemMode;
import org.aluminati3555.lib.trajectoryfollowingmotion.AluminatiRobotStateEstimator;
import org.aluminati3555.lib.trajectoryfollowingmotion.RobotState;
import org.aluminati3555.lib.util.AluminatiUtil;
import org.aluminati3555.lib.vision.AluminatiLimelight;
import org.aluminati3555.lib.vision.AluminatiLimelight.LEDMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import org.aluminati3555.frc2020.auto.actions.ActionAlignWithVision;
import org.aluminati3555.frc2020.auto.actions.ActionTurnToHeading;
import org.aluminati3555.frc2020.auto.modes.Mode10PowerCell;
import org.aluminati3555.frc2020.auto.modes.Mode3PowerCellGoForward;
import org.aluminati3555.frc2020.auto.modes.Mode5PowerCell2OtherAllianceTrenchRun;
import org.aluminati3555.frc2020.auto.modes.Mode5PowerCell2ShieldGenerator;
import org.aluminati3555.frc2020.auto.modes.Mode5PowerCell3ShieldGenerator;
import org.aluminati3555.frc2020.auto.modes.Mode8PowerCell5TrenchRun;
import org.aluminati3555.frc2020.auto.modes.ModeCharacterizeDrive;
import org.aluminati3555.frc2020.auto.modes.ModeDoNothing;
import org.aluminati3555.frc2020.auto.modes.ModeExamplePath;
import org.aluminati3555.frc2020.auto.modes.ModeExampleTurn;
import org.aluminati3555.frc2020.auto.modes.ModeGoForward;
import org.aluminati3555.frc2020.systems.DriveSystem;
import org.aluminati3555.frc2020.systems.MagazineSystem;
import org.aluminati3555.frc2020.systems.IntakeSystem;
import org.aluminati3555.frc2020.systems.ShooterSystem;
import org.aluminati3555.frc2020.systems.SpinnerSystem;
import org.aluminati3555.frc2020.util.ShooterUtil;

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

  // Digit display
  private AluminatiDisplay display;

  // Joysticks
  private AluminatiXboxController driverController;
  private AluminatiXboxController operatorController;

  // Limelight
  private AluminatiLimelight limelight;

  // System faults
  RobotFaults robotFaults;

  // Systems
  private DriveSystem driveSystem;
  private SpinnerSystem spinnerSystem;
  private ShooterSystem shooterSystem;
  private IntakeSystem intakeSystem;
  private MagazineSystem magazineSystem;

  // Pneumatics
  private AluminatiCompressor compressor;

  // Auto selector
  private AluminatiAutoSelector autoSelector;

  // Video display
  private VideoDisplay videoDisplay;

  @Override
  public void robotInit() {
    // Configure pid
    AluminatiData.velocityKF = 0.277254831449;
    AluminatiData.velocityKP = 0.2;
    AluminatiData.velocityKI = 0.0001;
    AluminatiData.velocityKD = 0.25;

    // Configure pure pursuit
    AluminatiData.pathFollowingProfileKP = 5;
    AluminatiData.pathFollowingProfileKI = 0.0001;
    AluminatiData.pathFollowingProfileKV = 0.007;
    AluminatiData.pathFollowingProfileKS = 0.076;

    AluminatiData.pathFollowingMaxVel = 150;
    AluminatiData.pathFollowingMaxAccel = 100;

    AluminatiUtil.generatePathFollowingFeedforwardValues();

    // Configure stop steering distance to be small since our paths require turning
    // at the end
    AluminatiData.pathStopSteeringDistance = 6;

    // Set encoder data
    AluminatiData.encoderUnitsPerRotation = 4096;

    // Set robot physical constants
    AluminatiData.wheelDiamater = 6;
    AluminatiData.driveWidth = 25.125;

    // Set thread priority
    Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
    Thread.currentThread().setName("Robot-Thread");

    // Set default robot state and mode
    robotState = new RobotState();
    robotMode = RobotMode.OPERATOR_CONTROLLED;

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

    // Setup digit display
    display = new AluminatiDisplay();

    // Setup joysticks
    driverController = new AluminatiXboxController(0);
    operatorController = new AluminatiXboxController(1);

    // Setup limelight
    limelight = new AluminatiLimelight();
    limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);
    limelight.setPipeline(0);

    robotFaults = new RobotFaults();

    // Configure systems
    configureSystems();

    // Setup pid tuning on actions
    ActionTurnToHeading.initialize();
    ActionAlignWithVision.initialize();

    // Load neural network
    if (!Robot.isSimulation()) {
      ShooterUtil.load(Filesystem.getDeployDirectory() + "/shooter.ml");
    }

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
        new Entry("8PowerCell5TrenchRun",
            new Mode8PowerCell5TrenchRun(robotState, limelight, driveSystem, intakeSystem, shooterSystem,
                magazineSystem)),
        new Entry("5PowerCell2ShieldGenerator",
            new Mode5PowerCell2ShieldGenerator(robotState, limelight, driveSystem, intakeSystem, shooterSystem,
                magazineSystem)),
        new Entry("5PowerCell3ShieldGenerator",
            new Mode5PowerCell3ShieldGenerator(robotState, limelight, driveSystem, intakeSystem, shooterSystem,
                magazineSystem)),
        new Entry("5PowerCell2OtherAllianceTrenchRun",
            new Mode5PowerCell2OtherAllianceTrenchRun(
                robotState, limelight, driveSystem, intakeSystem, shooterSystem, magazineSystem)),
        new Entry("GoForward", new ModeGoForward(robotState, driveSystem)),
        new Entry("3PowerCellGoForward",
            new Mode3PowerCellGoForward(robotState, limelight, driveSystem, intakeSystem, shooterSystem,
                magazineSystem)),
        new Entry("10PowerCell",
            new Mode10PowerCell(robotState, limelight, driveSystem, intakeSystem, shooterSystem, magazineSystem)),
        new Entry("ExampleTurn", new ModeExampleTurn(robotState, driveSystem)));

    // Setup video display
    videoDisplay = new VideoDisplay("VideoDisplay", 3);
  }

  @Override
  public void robotPeriodic() {
    updateDisplay();

    Pose2d position = robotState.getLatestFieldToVehicle().getValue();
    Translation2d translation = position.getTranslation();
    Rotation2d rotation = position.getRotation();

    AluminatiAutoTask auto = autoSelector.getSelected();
    String autoString = (auto == null) ? "DoNothing" : auto.toString();

    videoDisplay.update(controlPanelColor, this.getAverageDT(), translation.x(), translation.y(), rotation.getDegrees(),
        autoString, robotFaults, limelight, shooterSystem.get(), shooterSystem.getVelocity(),
        magazineSystem.getFeederVelocity(), robotMode);
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    // Use brake mode if connected to the driverstation/fms
    if (DriverStation.getInstance().isDSAttached() || DriverStation.getInstance().isFMSAttached()) {
      driveSystem.brake();

      // Turn limelight to current pipeline
      limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);
    } else {
      // Use coast for vision calibration
      driveSystem.coast();

      // Have limelight on for vision calibration
      limelight.setLEDMode(LEDMode.ON);
    }

    // Zero gyro if waiting for match to start
    if (!matchStarted) {
      driveSystem.getGyro().setHeading(Rotation2d.fromDegrees(0));
    }

    // Update systems
    double timestamp = Timer.getFPGATimestamp();
    driveSystem.update(timestamp, SystemMode.DISABLED);
    spinnerSystem.update(timestamp, SystemMode.DISABLED);
    shooterSystem.update(timestamp, SystemMode.DISABLED);
    intakeSystem.update(timestamp, SystemMode.DISABLED);
    magazineSystem.update(timestamp, SystemMode.DISABLED);
  }

  @Override
  public void autonomousInit() {
    matchStarted = true;

    // Set brake mode
    driveSystem.brake();

    // Set limelight mode
    limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);

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

    // Set limelight mode
    limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);

    double timestamp = Timer.getFPGATimestamp();

    autoControl(timestamp);

    // Update systems
    driveSystem.update(timestamp, SystemMode.AUTONOMOUS);
    spinnerSystem.update(timestamp, SystemMode.AUTONOMOUS);
    shooterSystem.update(timestamp, SystemMode.AUTONOMOUS);
    intakeSystem.update(timestamp, SystemMode.AUTONOMOUS);
    magazineSystem.update(timestamp, SystemMode.AUTONOMOUS);
  }

  @Override
  public void teleopInit() {
    matchStarted = true;

    // Set brake mode
    driveSystem.brake();

    // Set limelight mode
    limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);
  }

  @Override
  public void teleopPeriodic() {
    SystemMode mode = (robotMode == RobotMode.OPERATOR_CONTROLLED) ? SystemMode.OPERATOR_CONTROLLED
        : SystemMode.AUTONOMOUS;

    // Set brake mode
    driveSystem.brake();

    // Set limelight mode
    limelight.setLEDMode(LEDMode.CURRENT_PIPELINE);

    // Fetch control panel color if we do not already have it
    if (controlPanelColor == ControlPanelColor.UNKOWN) {
      fetchControlPanelColor();
    }

    double timestamp = Timer.getFPGATimestamp();

    autoControl(timestamp);

    // Update systems
    driveSystem.update(timestamp, mode);
    spinnerSystem.update(timestamp, mode);
    shooterSystem.update(timestamp, mode);
    intakeSystem.update(timestamp, mode);
    magazineSystem.update(timestamp, mode);
  }

  @Override
  public void testInit() {
    // Mark match as started for testing
    matchStarted = true;

    // Set coast mode
    driveSystem.coast();
  }

  @Override
  public void testPeriodic() {
    // Set coast mode
    driveSystem.coast();
  }

  /**
   * Configures the robot systems
   */
  private void configureSystems() {
    // Setup drivetrain
    AluminatiMotorGroup left = new AluminatiMotorGroup(new AluminatiTalonSRX(35), new AluminatiVictorSPX(14),
        new AluminatiVictorSPX(15));
    AluminatiMotorGroup right = new AluminatiMotorGroup(true, new AluminatiTalonSRX(50), new AluminatiVictorSPX(51),
        new AluminatiVictorSPX(1));

    left.getMasterTalon().setSensorPhase(true);
    right.getMasterTalon().setSensorPhase(true);

    AluminatiPigeon gyro = new AluminatiPigeon(new AluminatiTalonSRX(43));

    AluminatiTalonSRX feederMotor = new AluminatiTalonSRX(46);

    driveSystem = new DriveSystem(looper, robotState, left, right, gyro, driverController, robotFaults);

    spinnerSystem = new SpinnerSystem(new AluminatiTalonSRX(60), new AluminatiSolenoid(0), operatorController,
        robotFaults);
    magazineSystem = new MagazineSystem(new AluminatiVictorSPX(7), feederMotor, robotFaults);
    shooterSystem = new ShooterSystem(new AluminatiMotorGroup(new AluminatiTalonSRX(23), new AluminatiTalonSRX(45)),
        new AluminatiSpark(0), driverController, operatorController, limelight, driveSystem, magazineSystem,
        robotFaults);
    intakeSystem = new IntakeSystem(new AluminatiTalonSRX(44), new AluminatiSolenoid(2), operatorController,
        magazineSystem, robotFaults);
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
   * Returns true if the auto kill switch is activated on the driver joystick
   */
  private boolean getAutoKillSwitch() {
    return (driverController.getRawButton(9) && driverController.getRawButton(10)
        && driverController.getX(Hand.kLeft) >= 0.5 && driverController.getX(Hand.kRight) <= 0.5);
  }

  /**
   * Controls the robot during auto
   */
  private void autoControl(double timestamp) {
    if (!DriverStation.getInstance().isAutonomous()) {
      if (getAutoKillSwitch()) {
        // Stop task if there is one
        if (autoTask != null) {
          autoTask.stop();
        }
        robotMode = RobotMode.OPERATOR_CONTROLLED;
      }
    }

    if (robotMode == RobotMode.AUTONOMOUS && autoTask != null) {
      if (autoTask.isComplete()) {
        // Stop task and cleanup
        autoTask.stop();
        robotMode = RobotMode.OPERATOR_CONTROLLED;
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

  /**
   * Updates the digit display
   */
  private void updateDisplay() {
    display.display(pdp.getVoltage());
  }

  public enum RobotMode {
    AUTONOMOUS, OPERATOR_CONTROLLED
  }

  public enum ControlPanelColor {
    UNKOWN, BLUE, GREEN, RED, YELLOW
  }

  private class DataReporter implements Loop {
    public void onStart(double timestamp) {

    }

    public void onLoop(double timestamp) {
      // Do not report data if connected to the fms
      if (!DriverStation.getInstance().isFMSAttached()) {
        SmartDashboard.putNumber("leftPower", driveSystem.getLeftGroup().getMasterTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("rightPower", driveSystem.getRightGroup().getMasterTalon().getMotorOutputPercent());
        SmartDashboard.putNumber("shooterPower", shooterSystem.getOutputPercent());
        SmartDashboard.putNumber("feederPower", magazineSystem.getFeederOutputPercent());
      }
    }

    public void onStop(double timestamp) {

    }

    public String getName() {
      return "[DataReporter]";
    }
  }
}
