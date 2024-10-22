// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;


public class AbsoluteDriveAdvHdgAim extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  oX, oY;
  private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight, lookTarget, hdgMode;

  boolean hdgModePressed = false; // Flag to track button state
  boolean angHdgMode = false; // Flag to track angle mode
  boolean angVelMode = true; // Flag to track velocity mode (default)
  boolean hdgPOV = false; // Flag to track POV mode
  boolean resetHeading = false; // Flag to track heading reset 

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. There is a mode button that will allow selection of the
   * heading to be controlled by angular heading or angular velocity. The look booleans are shortcuts to get the robot to
   * face a certian direction. With an extra look boolean that faces the robot towards the intended vision target.
   * This is a mashup of driveFieldOrientedAnglularVelocity and AbsoluteDriveAdv. 
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param oX            DoubleSupplier that supplies the x-orientation joystick input.  Should be in the range -1 to 1
   * @param oY            DoubleSupplier that supplies the y-orientation joystick input.  Should be in the range -1 to 1
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   * @param lookTarget    Face the robot towards the vision target
   * @param hdgMode       Switch between angle and velocity mode
   */
  public AbsoluteDriveAdvHdgAim(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier oX, DoubleSupplier oY,
                          BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                          BooleanSupplier lookRight, BooleanSupplier lookTarget, BooleanSupplier hdgMode)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.oX = oX;
    this.oY = oY;
    this.lookAway = lookAway;
    this.lookTowards = lookTowards;
    this.lookLeft = lookLeft;
    this.lookRight = lookRight;
    this.lookTarget = lookTarget;
    this.hdgMode = hdgMode;
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    resetHeading = true;
    angVelMode = true;
    hdgPOV = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double headingX = oX.getAsDouble();
    double headingY = oY.getAsDouble();

    if (hdgMode.getAsBoolean() && !hdgModePressed) {
      hdgModePressed = true; // Button pressed, set flag to true

      if (angHdgMode) {
        angVelMode = true; // Switch to velocity mode
        angHdgMode = false; // Switch off angle mode
      } else {
        angHdgMode = true; // Switch to angle mode
        angVelMode = false; // Switch off velocity mode
        // This is to prevent the robot from spinning when switching modes
        Rotation2d currentHeading = swerve.getHeading(); // Get current heading
        headingX = currentHeading.getSin(); // Set headingX to sin of current heading
        headingY = currentHeading.getCos(); // Set headingY to cos of current heading
      }
      //System.out.println("hdgMode: " + hdgMode.getAsBoolean() + " angHdgMode: " + angHdgMode + " angVelMode: " + angVelMode); // Debugging
    } else if (!hdgMode.getAsBoolean()) {
      hdgModePressed = false; // Button released, reset flag
    }
    
    // Face Away from Drivers
    if (lookAway.getAsBoolean())
    {
      headingY = -1;
      hdgPOV = true;
    }

    // Face Right
    if (lookRight.getAsBoolean())
    {
      headingX = 1;
      hdgPOV = true;
    }
    // Face Left
    if (lookLeft.getAsBoolean())
    {
      headingX = -1;
      hdgPOV = true;
    }
    // Face Towards the Drivers
    if (lookTowards.getAsBoolean())
    {
      headingY = 1;
      hdgPOV = true;
    }
    
    // Face Towards the targte, speaker in cressendo game
    if (lookTarget.getAsBoolean())
    {
      headingX = swerve.getSpeakerYaw().getSin(); // Get the sin of the speaker yaw
      headingY = swerve.getSpeakerYaw().getCos(); // Get the cos of the speaker yaw
    }
    
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY); // Get the target speeds

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                          Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                          swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    if (angHdgMode == true || lookTarget.getAsBoolean() == true) // If in angle mode or looking at the target
    {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true); // Drive with the desired speeds
    }
    if (angVelMode == true && lookTarget.getAsBoolean() == false) // If in velocity mode and not looking at the target
    {
      swerve.drive(translation, MathUtil.applyDeadband(-oX.getAsDouble() * 8.0, OperatorConstants.RIGHT_X_DEADBAND), true); // Drive with the desired speeds
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

}
