// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	private final PhotonCamera m_camera = new PhotonCamera("camera");
	private final PhotonCameraSim m_cameraSim;
	private final VisionSystemSim m_visionSystemSim;
	private final double kCameraHeight = 0.14;
	private final double kTargetHeight = 0.5;
	private final PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator(
			AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			m_camera, new Transform3d(0.5, 0.5, kCameraHeight, new Rotation3d()));

	/** Creates a new Vision object. */
	public Vision() {
		if (RobotBase.isSimulation()) {
			m_cameraSim = new PhotonCameraSim(m_camera, new SimCameraProperties());
			m_visionSystemSim = new VisionSystemSim("main");
			m_visionSystemSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
			m_visionSystemSim.addCamera(m_cameraSim, new Transform3d(0.5, 0.5, kCameraHeight, new Rotation3d()));
			m_cameraSim.enableRawStream(true);
			m_cameraSim.enableProcessedStream(true);
			m_cameraSim.enableDrawWireframe(true);
		} else {
			m_cameraSim = null;
			m_visionSystemSim = null;
		}
	}

	public void updateVisionSim(Pose2d pose) {
		m_visionSystemSim.update(pose);
	}

	public Optional<EstimatedRobotPose> getVisionPose() {
		return m_poseEstimator.update();
	}
}
