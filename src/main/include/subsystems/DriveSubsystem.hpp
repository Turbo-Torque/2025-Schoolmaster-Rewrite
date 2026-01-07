// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>
#include <random>

#include "constants/Constants.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/RobotBase.h"
#include "frc/controller/PIDController.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SendableChooser.h"
#include "frc2/command/SubsystemBase.h"
#include "networktables/StructArrayTopic.h"
#include "networktables/StructTopic.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "motors/NeoKrakenModule.hpp"
#include "utils/TurboPoseEstimator.hpp"

class DriveSubsystem final : public frc2::SubsystemBase {
  NeoKrakenModule fleft, fright, bleft, bright;
  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveSubsystemConstants::kPigeonID, DriveSubsystemConstants::kCanivoreName};
  frc::Rotation2d driverGyroOffset{};
  TurboPoseEstimator estimator;

  frc::ChassisSpeeds m_cmdSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};
  frc::Pose2d m_simPose = frc::Pose2d(9.506_m, 4.067_m, 0_deg);
  units::second_t m_lastTime{0.0_s};
  frc::Pose2d goalPose{};

  std::mt19937 gen;
  std::uniform_real_distribution<> x_dist;
  std::uniform_real_distribution<> y_dist;
  std::uniform_real_distribution<> theta_dist;

  nt::StructPublisher<frc::Pose2d> m_posePublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_speedsPublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_cmdSpeedsPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_swerveStatesPublisher;
  nt::StructPublisher<frc::Pose2d> m_goalPosePublisher;

  nt::StructTopic<frc::ChassisSpeeds> m_dynamicsBestChassisSpeedsTopic;

  frc::SendableChooser<int> simulationBehaviorChooser;

  frc::PIDController xController{2, 0.0, 0.0};
  frc::PIDController yController{2, 0.0, 0.0};
  frc::PIDController thetaController{0.05, 0.0, 0.0};

 public:
  DriveSubsystem();

  void Drive(const frc::ChassisSpeeds& speeds);
  void AutoDrive(frc::ChassisSpeeds speeds);
  void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);
  void DriverGyroZero();
  frc::Rotation2d GetDriverGyroAngle() const;
  frc::Rotation2d GetAngle() const { return pigeon.GetRotation2d(); }
  std::array<frc::SwerveModuleState, 4> GetModuleStates();
  std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  frc::Pose2d GetPose() const {
    if constexpr (frc::RobotBase::IsReal()) {
      return estimator.GetPose2D();
    } else {
      return m_simPose;
    }
  }

  frc::ChassisSpeeds GetRobotRelativeChassisSpeeds() {
    if constexpr (frc::RobotBase::IsReal()) {
      return DriveSubsystemConstants::kKinematics.ToChassisSpeeds(GetModuleStates());
    } else {
      return m_cmdSpeeds;
    }
  }

  TurboPoseEstimator& GetPoseEstimator() { return estimator; }

  frc2::CommandPtr DriveCommand(std::function<double()> xSpeed, std::function<double()> ySpeed,
                                std::function<double()> rotSpeed);

  void Periodic() override;
  void SimulationPeriodic() override;
};
