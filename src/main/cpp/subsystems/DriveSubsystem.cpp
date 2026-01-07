// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/DriveSubsystem.hpp"

#include <algorithm>
#include <array>

#include "constants/Constants.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SendableChooser.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StructArrayTopic.h"
#include "units/length.h"
#include "units/velocity.h"
#include "utils/TurboPoseEstimator.hpp"
#include <random>

#include <frc/DataLogManager.h>
#include <frc2/command/FunctionalCommand.h>

DriveSubsystem::DriveSubsystem()
    : fleft(DriveSubsystemConstants::kFrontLeftDriveID, DriveSubsystemConstants::kFrontLeftSteerID,
            DriveSubsystemConstants::kFrontLeftEncoderID, DriveSubsystemConstants::kFrontLeftOffset,
            DriveSubsystemConstants::kCanivoreName),
      fright(DriveSubsystemConstants::kFrontRightDriveID, DriveSubsystemConstants::kFrontRightSteerID,
             DriveSubsystemConstants::kFrontRightEncoderID, DriveSubsystemConstants::kFrontRightOffset,
             DriveSubsystemConstants::kCanivoreName),
      bleft(DriveSubsystemConstants::kBackLeftDriveID, DriveSubsystemConstants::kBackLeftSteerID,
            DriveSubsystemConstants::kBackLeftEncoderID, DriveSubsystemConstants::kBackLeftOffset,
            DriveSubsystemConstants::kCanivoreName),
      bright(DriveSubsystemConstants::kBackRightDriveID, DriveSubsystemConstants::kBackRightSteerID,
             DriveSubsystemConstants::kBackRightEncoderID, DriveSubsystemConstants::kBackRightOffset,
             DriveSubsystemConstants::kCanivoreName),
      estimator(GetAngle(), GetModulePositions(), frc::Pose2d(9.506_m, 4.067_m, 0_deg)) {
  // Generate a random initial pose for simulation or estimator
  static std::random_device rd;
  gen = std::mt19937(rd());
  x_dist = std::uniform_real_distribution<>(0.0, 16.0);
  y_dist = std::uniform_real_distribution<>(0.0, 8.0);
  theta_dist = std::uniform_real_distribution<>(0.0, 360.0);

  const auto rand_x = units::meter_t{x_dist(gen)};
  const auto rand_y = units::meter_t{y_dist(gen)};
  const auto rand_theta = frc::Rotation2d(units::degree_t{theta_dist(gen)});

  m_simPose = frc::Pose2d(rand_x, rand_y, rand_theta);

  goalPose = frc::Pose2d(units::meter_t{x_dist(gen)}, units::meter_t{y_dist(gen)},
                         frc::Rotation2d(units::degree_t{theta_dist(gen)}));

  m_posePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Publish();
  m_speedsPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("Speeds").Publish();
  m_swerveStatesPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("SwerveStates").Publish();
  m_cmdSpeedsPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("CmdSpeeds").Publish();
  m_dynamicsBestChassisSpeedsTopic =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("BestAction");
  m_goalPosePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("GoalPose").Publish();

  m_posePublisher.Set(frc::Pose2d());
  m_speedsPublisher.Set(frc::ChassisSpeeds());
  m_swerveStatesPublisher.Set(GetModuleStates());
  m_cmdSpeedsPublisher.Set(frc::ChassisSpeeds());
  m_goalPosePublisher.Set(goalPose);

  m_lastTime = frc::Timer::GetFPGATimestamp();

  simulationBehaviorChooser = frc::SendableChooser<int>();
  simulationBehaviorChooser.SetDefaultOption("Teleoperated", 0);
  simulationBehaviorChooser.AddOption("Teleoperated", 0);
  simulationBehaviorChooser.AddOption("CEM Controlled", 1);
  simulationBehaviorChooser.AddOption("PID Controlled", 2);

  thetaController.EnableContinuousInput(-180.0, 180.0);

  frc::SmartDashboard::PutData("Simulation Behavior", &simulationBehaviorChooser);

  SetName("DriveSubsystem");
}

void DriveSubsystem::Drive(const frc::ChassisSpeeds& speeds) {
  m_cmdSpeeds = speeds;
  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  SetModuleStates(states);

  m_swerveStatesPublisher.Set(states);
  m_speedsPublisher.Set(speeds);
}

void DriveSubsystem::AutoDrive(frc::ChassisSpeeds speeds) {
  m_cmdSpeeds = speeds;

  speeds.vx *= -1;
  speeds.vy *= -1;
  speeds.omega *= -1;

  const auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  SetModuleStates(states);

  // undo negation to publish correct speeds
  auto normalStates = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(m_cmdSpeeds);
  m_swerveStatesPublisher.Set(normalStates);
  m_speedsPublisher.Set(m_cmdSpeeds);
}

void DriveSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
  fleft.SetModuleState(states[0]);
  fright.SetModuleState(states[1]);
  bleft.SetModuleState(states[2]);
  bright.SetModuleState(states[3]);
}

void DriveSubsystem::DriverGyroZero() {
  driverGyroOffset = GetAngle();
}

std::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return {fleft.GetModuleState(), fright.GetModuleState(), bleft.GetModuleState(), bright.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {fleft.GetModulePosition(), fright.GetModulePosition(), bleft.GetModulePosition(), bright.GetModulePosition()};
}

frc::Rotation2d DriveSubsystem::GetDriverGyroAngle() const {
  return GetAngle() - driverGyroOffset;
}

frc2::CommandPtr DriveSubsystem::DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed,
                                              std::function<double()> rotSpeed) {
  return frc2::FunctionalCommand(
             [] {},
             [=, this] {
               if (simulationBehaviorChooser.GetSelected() != 0) {
                 return;
               }

               const frc::ChassisSpeeds cmdSpeeds =
                   frc::ChassisSpeeds{ySpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                                      xSpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                                      rotSpeed() * DriveSubsystemConstants::kMaxAngularSpeed};

               m_cmdSpeedsPublisher.Set(cmdSpeeds);

               const frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                   ySpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                   xSpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                   rotSpeed() * DriveSubsystemConstants::kMaxAngularSpeed, GetDriverGyroAngle());

               Drive(fieldRelativeSpeeds);
             },
             [](bool) {}, [] { return false; }, {this})
      .ToPtr()
      .WithName("DriveCommand");
}

void DriveSubsystem::Periodic() {
  if constexpr (frc::RobotBase::IsReal()) {
    estimator.UpdateWithOdometryAndVision(GetAngle(), GetModulePositions());

    m_posePublisher.Set(estimator.GetPose2D());
  }

  frc::SmartDashboard::PutNumber("Gyro", GetAngle().Degrees().value());
}

void DriveSubsystem::SimulationPeriodic() {
  const auto currentTime = frc::Timer::GetFPGATimestamp();
  const auto dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  if (m_simPose.Translation().Distance(goalPose.Translation()) < 0.1_m &&
      std::abs((m_simPose.Rotation() - goalPose.Rotation()).Radians().value()) < 0.1) {
    goalPose = frc::Pose2d(units::meter_t{x_dist(gen)}, units::meter_t{y_dist(gen)},
                           frc::Rotation2d(units::degree_t{theta_dist(gen)}));
    m_goalPosePublisher.Set(goalPose);
  }

  units::velocity::meters_per_second_t vx;
  units::velocity::meters_per_second_t vy;
  units::angular_velocity::radians_per_second_t omega;

  if (simulationBehaviorChooser.GetSelected() == 0) {
    auto cmdSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_cmdSpeeds, m_simPose.Rotation());
    vx = cmdSpeeds.vx;
    vy = cmdSpeeds.vy;
    omega = cmdSpeeds.omega;
  } else if (simulationBehaviorChooser.GetSelected() == 1) {
    auto best_chassis_speeds = m_dynamicsBestChassisSpeedsTopic.GetEntry({}).Get();
    vx = best_chassis_speeds.vx;
    vy = best_chassis_speeds.vy;
    omega = best_chassis_speeds.omega;

    m_cmdSpeedsPublisher.Set(best_chassis_speeds);
  } else if (simulationBehaviorChooser.GetSelected() == 2) {
    vx = xController.Calculate(m_simPose.X().value(), goalPose.X().value()) * 1_mps;
    vy = yController.Calculate(m_simPose.Y().value(), goalPose.Y().value()) * 1_mps;
    omega = thetaController.Calculate(m_simPose.Rotation().Degrees().value(), goalPose.Rotation().Degrees().value()) *
            1_rad_per_s;

    vx = std::clamp(vx, -DriveSubsystemConstants::kMaxLinearSpeed, DriveSubsystemConstants::kMaxLinearSpeed);
    vy = std::clamp(vy, -DriveSubsystemConstants::kMaxLinearSpeed, DriveSubsystemConstants::kMaxLinearSpeed);
    omega = std::clamp(omega, -DriveSubsystemConstants::kMaxAngularSpeed, DriveSubsystemConstants::kMaxAngularSpeed);

    m_cmdSpeedsPublisher.Set(frc::ChassisSpeeds{vx, vy, omega});
  }

  const auto newX = m_simPose.X() + vx * dt;
  const auto newY = m_simPose.Y() + vy * dt;
  const auto newTheta = frc::Rotation2d(m_simPose.Rotation().Radians() + omega * dt);
  m_simPose = frc::Pose2d(newX, newY, newTheta);

  m_posePublisher.Set(m_simPose);

  auto& simGyro = pigeon.GetSimState();
  simGyro.SetRawYaw(m_simPose.Rotation().Degrees());
}
