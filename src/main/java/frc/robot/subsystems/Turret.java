// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import frc.robot.subsystems.PhotonVision;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CustomXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private SparkFlexConfig turretConfig;
  private RelativeEncoder turretEncoder;
  private DutyCycleEncoder throughBore;
  private SparkClosedLoopController turretController;

  private SwerveDrive swerve;
	private PhotonVision vision;
  private Shooter shooter;

  private double goal;
  private boolean manual;
	private boolean fastTurret;

  /** Creates a new Turret. */
  public Turret(SwerveDrive swerve, PhotonVision vision, Shooter shooter) {
    this.swerve = swerve;
		this.vision = vision;
    this.shooter = shooter;

    turretMotor = new SparkFlex(TurretConstants.TURRET_ID, MotorType.kBrushless);
    turretConfig = new SparkFlexConfig();
    throughBore = new DutyCycleEncoder(0);
    turretEncoder = turretMotor.getEncoder();
    turretController = turretMotor.getClosedLoopController();

    turretConfig
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    turretConfig.encoder
      .positionConversionFactor(TurretConstants.TURRET_POSITION_CONVERSION);

    turretConfig.closedLoop
      .p(TurretConstants.TURRET_PID_VALUES_SLOT0[0], ClosedLoopSlot.kSlot0)
      .i(TurretConstants.TURRET_PID_VALUES_SLOT0[1], ClosedLoopSlot.kSlot0)
      .d(TurretConstants.TURRET_PID_VALUES_SLOT0[2], ClosedLoopSlot.kSlot0)
			.p(TURRET_PID_VALUES_SLOT1[0], ClosedLoopSlot.kSlot1)
			.i(TURRET_PID_VALUES_SLOT1[1], ClosedLoopSlot.kSlot1)
			.d(TURRET_PID_VALUES_SLOT1[2], ClosedLoopSlot.kSlot1)
      .positionWrappingEnabled(false)
			.allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .allowedClosedLoopError(TURRET_ALLOWED_ERROR, ClosedLoopSlot.kSlot1);

    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    manual = false;
		fastTurret = false;
    goal = 0.0;
  }

  public void trackPose(Pose2d pose, CustomXboxController controller) {
    if(!manual){
			double safeGoal = MathUtil.clamp(angleToPose(pose), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
      goal = safeGoal;
			if(turretEncoder.getPosition() < -40) {
				turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}else if(turretEncoder.getPosition() > goal) {
    		turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
			}else{
				turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}
    }else{
      manualTurret(controller);
    }
  }

  public void trackLookAheadPose(Pose2d pose, CustomXboxController controller) {
    if(!manual){
			double safeGoal = MathUtil.clamp(angleToLookAheadPose(pose), TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
      goal = safeGoal;
			if(turretEncoder.getPosition() < -40) {
				turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}else if(turretEncoder.getPosition() > goal) {
    		turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
			}else{
				turretController.setSetpoint(goal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}
    }else{
      manualTurret(controller);
    }
  }

  /**Rotates a the turret to an angle when not on manual control; otherwise, it is controlled with a Joystick */
  public void turretToAngle(double angleToHub, CustomXboxController controller){
    if(!manual){
			double safeGoal = -MathUtil.clamp(goal, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE);
			if(turretEncoder.getPosition() < -40) {
				turretController.setSetpoint(safeGoal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}else if(Math.abs(turretEncoder.getPosition()) > safeGoal) {
    		turretController.setSetpoint(safeGoal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
			}else if(Math.abs(turretEncoder.getPosition()) < safeGoal) {
				turretController.setSetpoint(safeGoal, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}
      // double currentPosition = turretEncoder.getPosition();
      // double convertedGoal = currentPosition + goal % 360;

      // if(convertedGoal > TurretConstants.TURRET_MAX_ANGLE || convertedGoal < TurretConstants.TURRET_MIN_ANGLE)
      //   convertedGoal = (convertedGoal > TurretConstants.TURRET_MAX_ANGLE) ? convertedGoal - 360 : convertedGoal + 360;
      
      // turretController.setSetpoint(convertedGoal, ControlType.kPosition);
    }
    else
      manualTurret(controller);
  }

  public void turnTurret(double setpoint, CustomXboxController controller) {
		if(!manual) {
			if(turretEncoder.getPosition() < -40) {
				turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}else if(Math.abs(turretEncoder.getPosition()) > setpoint) {
    		turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
			}else if(Math.abs(turretEncoder.getPosition()) < setpoint) {
				turretController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
			}
		}else
			manualTurret(controller);
  }

  /**Controls the turret with joystick inputs */
  public void manualTurret(CustomXboxController controller){
		double speed = fastTurret ? controller.getRawAxis(4) / 5 : controller.getRawAxis(4) / 10;

    if(turretEncoder.getPosition() >= TurretConstants.TURRET_MAX_ANGLE && speed > 0)
      stop();

    else if(turretEncoder.getPosition() <= TurretConstants.TURRET_MIN_ANGLE && speed < 0)
      stop();
    
    else
      turretMotor.set(speed);
  }

  /**Sets the turret motor speed to 0 */
  public void stop(){
    turretMotor.set(0);
  }

  /**Toggles whether or not the turret is manual */
  public void toggleManual(){
    manual = !manual;
  }

	/**Toggles whether the turret is on fast mode or slow mode.
	 * 
	 */
	public void toggleFastTurret() {
		fastTurret = !fastTurret;
	}

  public Pose2d[] getLookAheadPoses() {
    Pose2d currentPose = swerve.getPose2d();

    double xDisplacement = swerve.getChassisSpeeds().vxMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));
    double yDisplacement = swerve.getChassisSpeeds().vyMetersPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));
    // double rotationDisplacement = swerve.getChassisSpeeds().omegaRadiansPerSecond * shooter.getFlightTime(vision.getDistanceToHub(currentPose));

    double displacedX = currentPose.getX() + xDisplacement;
    double displacedY = currentPose.getY() + yDisplacement;
    // double displacedRotation = currentPose.getRotation().getRadians() + rotationDisplacement;

    Pose2d nextRobotPose = new Pose2d(displacedX, displacedY, new Rotation2d(currentPose.getRotation().getRadians()));
    Pose2d nextTurretPose = nextRobotPose.plus(TURRET_OFFSET);

    Pose2d[] lookAheadPoses = {nextRobotPose, nextTurretPose};

    return lookAheadPoses;
  }

	/**Returns whether the turret is on fast mode or not.
	 * 
	 * @return {@code true} if fast mode, {@code false} otherwise.
	 */
	public boolean turretIsFast() {
		return fastTurret;
	}

  public double angleToLookAheadPose(Pose2d pose) {
    double yDistance = pose.getY() - getLookAheadPoses()[1].getY();
    double xDistance = pose.getX() - getLookAheadPoses()[1].getX();

    Rotation2d angleToTarget = Rotation2d.fromRadians(Math.atan2(yDistance, xDistance));

    double ccwDesiredTurretAngle = -(180 + (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180)));
    double cwDesiredTurretAngle = 180 - (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(getLookAheadPoses()[0].getRotation().getDegrees(), -180, 180));

    double ccwAngleDistance = Math.abs(0 - ccwDesiredTurretAngle);
    double cwAngleDistance = Math.abs(0 - cwDesiredTurretAngle);

    if(ccwDesiredTurretAngle < TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = cwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle > TURRET_MAX_ANGLE) {
      goal = ccwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = ccwAngleDistance < cwAngleDistance ? ccwDesiredTurretAngle : cwDesiredTurretAngle;
    }else{
      goal = turretEncoder.getPosition();
    }

    return goal;
  }

  public double angleToPose(Pose2d pose) {
    Pose2d robotPose = swerve.getPose2d();
    // System.out.println("rPose: " + robotPose);
    Pose2d turretPose = robotPose.plus(TurretConstants.TURRET_OFFSET);
    // System.out.println("tPose: " + turretPose);

    double yDistance = pose.getY() - turretPose.getY();
    double xDistance = pose.getX() - turretPose.getX();
    // System.out.println("y: " + yDistance);
    // System.out.println("x: " + xDistance);

    Rotation2d angleToTarget = Rotation2d.fromRadians(Math.atan2(yDistance, xDistance));
    // System.out.println(angleToTarget);
    double ccwDesiredTurretAngle = -(180 + (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(robotPose.getRotation().getDegrees(), -180, 180)));
    // System.out.println("ccwDesiredTurretAngle: " + ccwDesiredTurretAngle);
    double cwDesiredTurretAngle = 180 - (MathUtil.inputModulus(angleToTarget.getDegrees(), -180, 180) - MathUtil.inputModulus(robotPose.getRotation().getDegrees(), -180, 180));
    // System.out.println("cwDesiredTurretAngle: " + cwDesiredTurretAngle);

    double ccwAngleDistance = Math.abs(0 - ccwDesiredTurretAngle);
    double cwAngleDistance = Math.abs(0 - cwDesiredTurretAngle);

    if(ccwDesiredTurretAngle < TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = cwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle > TURRET_MAX_ANGLE) {
      goal = ccwDesiredTurretAngle;
    }else if(ccwDesiredTurretAngle > TURRET_MIN_ANGLE && cwDesiredTurretAngle < TURRET_MAX_ANGLE) {
      goal = ccwAngleDistance < cwAngleDistance ? ccwDesiredTurretAngle : cwDesiredTurretAngle;
    }else{
      goal = turretEncoder.getPosition();
    }

    return goal;
  }

	/**Returns whether the turret is on manual mode or not.
	 * 
	 * @return {@code true} if manual mode, {@code false} otherwise.
	 */
	public boolean isManual() {
		return manual;
	}

  /**
   * @return Y coordinate of the turret on the field
   */
  public double getTurretY(){
    double h = swerve.getPose2d().getY(); //Robot Y coordinate
    double k = swerve.getPose2d().getX(); //Robot X coordinate
    double robotAngle = ((swerve.getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation
    double y = h + TurretConstants.TURRET_OFFSET_Y;
    double x = k + TurretConstants.TURRET_OFFSET_X;
    return h +((y-h)*Math.cos(robotAngle)) - ((x-k) * Math.sin(robotAngle));
  }

  /**
   * @return X coordinate of the turret on the field
   */
  public double getTurretX(){
    double h = swerve.getPose2d().getY(); //Robot y coordinate
    double k = swerve.getPose2d().getX(); //Robot x coordinate
    double robotAngle = ((swerve.getGyro().getYaw() % 360.0) + 360) % 360; //Robot rotation in degrees
    double y = h + TurretConstants.TURRET_OFFSET_Y; //Original y coordinate of turret relative to robot center y coordinate
    double x = k + TurretConstants.TURRET_OFFSET_X; //Original x coordinate of turret relative to robot center x coordinate
    return k +((y-h)*Math.sin(robotAngle)) + ((x-k) * Math.cos(robotAngle));
  }

  /**
   * @param pose Pose to get the angle to
   * @return angle to the pose
   */
  public double getAngleToPose(Pose2d pose){
    double x = pose.getX() - getTurretX();
    double y = pose.getY() - getTurretY();
		// System.out.println(x);
		// System.out.println(y);
		double rawAngleRad = Math.atan(x/y) * (180/Math.PI);
    return -(((((Math.atan(x/y)) + (swerve.gyroRad()+Math.PI)% (2* Math.PI)) * 180/Math.PI)-360) % 180);
		// return rawAngleRad;
  }

  /**puts the data for the turret on smartdashboard*/
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Turret Position", () -> turretEncoder.getPosition(), null);
    builder.addDoubleProperty("Turret PID Setpoint", () -> turretController.getSetpoint(), null);
    builder.addDoubleProperty("Turret Assumed Goal", () -> this.goal, null);
    builder.addDoubleProperty("Turret Absolute Position", () -> throughBore.get(), null);
		builder.addDoubleProperty("Angle To Hub", () -> getAngleToPose(vision.getHubCenterPose2d()), null);
    builder.addBooleanProperty("Throughbore Connected", () -> throughBore.isConnected(), null);
    // builder.addDoubleProperty("Turret X", () -> getTurretX(), null);
    // builder.addDoubleProperty("Turret Y", () -> getTurretY(), null);
		builder.addBooleanProperty("Manual Turret", () -> isManual(), null);
		builder.addBooleanProperty("Fast Turret", () -> turretIsFast(), null);
    builder.addDoubleProperty("Desired Turret Angle", () -> angleToPose(vision.getHubCenterPose2d()), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // vision.setTurretPose2d(getTurretX(), getTurretY(), turretEncoder.getPosition());
  }
}
