package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.auto.Alliance;

@Autonomous(name = "Pedro Pathing Autonomous 2", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous2 extends OpMode {

  private static final double FIELD_WIDTH_IN = 144.0;

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  private Alliance activeAlliance = Alliance.BLUE;
  private Timer pathTimer;

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    pathTimer = new Timer();

    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(poseForAlliance(new Pose(72, 8, Math.toRadians(90)), activeAlliance));

    paths = new Paths(follower, activeAlliance); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {

    public final PathChain Path1;
    public final PathChain Path2;
    public final PathChain Path3;
    public final PathChain Path4;

    public Paths(Follower follower, Alliance alliance) {
      Pose path1Start = poseForAlliance(new Pose(56.000, 8.000, Math.toRadians(90)), alliance);
      Pose path1End = poseForAlliance(new Pose(56.279, 19.817, Math.toRadians(109)), alliance);
      Pose path2End = poseForAlliance(new Pose(23.780, 23.780, Math.toRadians(90)), alliance);
      Pose path3End = poseForAlliance(new Pose(23.516, 39.633, Math.toRadians(90)), alliance);
      Pose path4End = poseForAlliance(new Pose(56.279, 19.552, Math.toRadians(109)), alliance);

      Path1 = follower
        .pathBuilder()
        .addPath(new BezierLine(path1Start, path1End))
        .setLinearHeadingInterpolation(path1Start.getHeading(), path1End.getHeading())
        .build();

      Path2 = follower
        .pathBuilder()
        .addPath(new BezierLine(path1End, path2End))
        .setLinearHeadingInterpolation(path1End.getHeading(), path2End.getHeading())
        .build();

      Path3 = follower
        .pathBuilder()
        .addPath(new BezierLine(path2End, path3End))
        .setTangentHeadingInterpolation()
        .build();

      Path4 = follower
        .pathBuilder()
        .addPath(new BezierLine(path3End, path4End))
        .setLinearHeadingInterpolation(path3End.getHeading(), path4End.getHeading())
        .build();
    }
  }

  public int autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(paths.Path1);
        setPathState(1);
        break;
      case 1:
        if (!follower.isBusy()) {
          follower.followPath(paths.Path2, true);
          setPathState(2);
        }
        break;
      case 2:
        if (!follower.isBusy()) {
          follower.followPath(paths.Path3, true);
          setPathState(3);
        }
        break;
      case 3:
        if (!follower.isBusy()) {
          follower.followPath(paths.Path4, true);
          setPathState(4);
        }
        break;
      case 4:
        if (!follower.isBusy()) {
          setPathState(-1);
        }
        break;
      default:
        break;
    }
    return pathState;
  }

  @Override
  public void init_loop() {
    Alliance desired = activeAlliance;
    if (gamepad1.x) {
      desired = Alliance.BLUE;
    } else if (gamepad1.b) {
      desired = Alliance.RED;
    }

    if (desired != activeAlliance) {
      activeAlliance = desired;
      follower.setStartingPose(poseForAlliance(new Pose(72, 8, Math.toRadians(90)), activeAlliance));
      paths = new Paths(follower, activeAlliance);
    }

    telemetry.addLine("Press X for Blue alliance or B for Red");
    telemetry.addData("Selected alliance", activeAlliance.displayName());
    telemetry.update();
  }

  private static Pose poseForAlliance(Pose bluePose, Alliance alliance) {
    if (alliance == Alliance.RED) {
      double mirroredX = FIELD_WIDTH_IN - bluePose.getX();
      double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - bluePose.getHeading());
      return new Pose(mirroredX, bluePose.getY(), mirroredHeading);
    }
    return bluePose;
  }

  public void setPathState(int state) {
    pathState = state;
    pathTimer.resetTimer();
  }

  @Override
  public void start() {
    setPathState(0);
  }
}
