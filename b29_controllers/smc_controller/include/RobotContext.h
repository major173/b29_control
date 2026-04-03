#pragma once
// ============================================================
// RobotContext.h
//
// 角色：
//   1. 作为 SMC 自动生成状态机的 owner（RobotFSMContext 的 owner）
//   2. 实现长期规范接口 robot_fsm::RobotActions
//   3. 提供当前 RobotFSM.sm 所需的兼容包装函数，避免立即修改 .sm
// ============================================================

#include <string>
#include <string_view>

#include "RobotActions.h"
#include "RobotFSM_sm.h"

class RobotContext : public robot_fsm::RobotActions {
public:
    RobotContext();
    virtual ~RobotContext() = default;

    RobotContext(const RobotContext&) = delete;
    RobotContext& operator=(const RobotContext&) = delete;

    // ---------------------- 对外：驱动状态机 ----------------------
    void start();
    void tick50Hz();

    RobotFSMContext& fsm() { return fsm_; }
    const RobotFSMContext& fsm() const { return fsm_; }

    // ---------------------- 规范接口实现 ----------------------
    bool isLowerAlive() const override;
    bool isImuReady() const override;
    bool isPostureReady() const override;
    bool isGripConfirmed() const override;

    bool isObstacleDetected() const override;
    bool isClassificationDone() const override;
    robot_fsm::ObstacleType getObstacleType() const override;
    robot_fsm::CrossingStrategy getObstacleStrategy() const override;

    bool isAtCrossingPosition() const override;
    bool isPostureAligned() const override;
    bool isCrossingStepDone() const override;
    bool isCrossingComplete() const override;
    bool isPostCheckPassed() const override;
    bool isPostCheckFailed() const override;

    double getCruiseSpeed() const override;
    double getApproachSpeed() const override;
    double getDamperCrossSpeed() const override;

    void stopAllMotors() override;
    void freezeAllJoints() override;
    void setTargetSpeed(double speed_mps) override;
    void setDriveMode(robot_fsm::DriveMode mode) override;
    void sendDriveCommand() override;
    void sendJointAngleCommand() override;

    void enableLineTracking() override;
    void disableLineTracking() override;
    void updateLineTrackingControl() override;

    void enableObstacleRanging() override;
    void disableObstacleRanging() override;
    void updateApproachControl() override;

    void startCrossingSequence(robot_fsm::CrossingStrategy strategy) override;
    void endCrossingSequence() override;
    void advanceCrossingStep() override;
    void enableJointMonitor() override;
    void disableJointMonitor() override;
    void setJointLimitCallback(const std::string& event_name) override;
    void checkGripForce() override;
    void loadCrossingPreset(robot_fsm::CrossingStrategy strategy) override;
    void confirmCrossingReady() override;

    void initAutoMode() override;
    void disableAutoMode() override;
    void startInitSequence() override;
    void clearInitFlags() override;
    void startPostureAlignment() override;
    void startPostCrossCheck() override;
    void clearObstacleState() override;

    void enableManualPassthrough() override;
    void disableManualPassthrough() override;

    void startObstacleClassification() override;
    void finalizeObstacleClass() override;
    void setObstacleStrategy(robot_fsm::CrossingStrategy strategy) override;
    void logObstacleFound() override;

    void startReconnectTimer() override;
    void stopReconnectTimer() override;

    void reportError(std::string_view reason) override;
    void reportCommsLoss() override;
    void reportEmergencyStop(std::string_view reason) override;
    void resetFaultFlags() override;
    void alertOperator(robot_fsm::AlertType type) override;
    void logSafeStopEntry() override;
    void logTransition(std::string_view info) override;

    // ---------------------- 当前 .sm 兼容包装 ----------------------
    bool canStartAuto() const;
    bool isReadyToTraverse() const;
    bool isObstacleNotDetected() const;
    bool isClassifiedAsLineClamp() const;
    bool isClassifiedAsDamper() const;
    bool isClassifiedAsUnknown() const;
    bool isNotAtCrossingPosition() const;
    bool isReadyForLineClamp() const;
    bool isReadyForDamper() const;
    bool isCrossingStepPending() const;

    void setDriveModeForward();
    void setTargetSpeedCruise();
    void setTargetSpeedZero();
    void setTargetSpeedApproach();
    void setTargetSpeedDamper();
    void setStrategyLineClamp();
    void setStrategyDamper();
    void loadCrossingPresetFromStrategy();
    void startCrossingLineClamp();
    void startCrossingDamper();

    void reportEmergencyStopManual();
    void reportEmergencyStopInit();
    void reportEmergencyStopTraversal();
    void reportEmergencyStopClassify();
    void reportEmergencyStopApproach();
    void reportEmergencyStopPreStop();
    void reportEmergencyStopLineClamp();
    void reportEmergencyStopDamper();
    void reportEmergencyStopPostCheck();

    void reportErrorAutoInitFailed();
    void reportErrorUnknownObstacle();
    void reportErrorClassifyTimeout();
    void reportErrorApproachTimeout();
    void reportErrorAlignTimeout();
    void reportErrorJointFaultLineClamp();
    void reportErrorImuFaultLineClamp();
    void reportErrorCrossingTimeoutLineClamp();
    void reportErrorJointFaultDamper();
    void reportErrorImuFaultDamper();
    void reportErrorGripFault();
    void reportErrorCrossingTimeoutDamper();
    void reportErrorPostCheckFailed();
    void reportErrorPostCheckTimeout();
    void reportErrorReconnectTimeout();

    void alertCommsLoss();
    void alertCommsRestored();
    void alertSafeStop();

    void logIdleToAutoInit();
    void logAutoInitToTraversing();
    void logTraversingPause();
    void logDetectedLineClamp();
    void logDetectedDamper();
    void logApproachingToPreStop();
    void logPreStopToLineClamp();
    void logPreStopToDamper();
    void logLineClampToPostCheck();
    void logDamperToPostCheck();
    void logPostCheckToTraversing();
    void logCommsRestored();
    void logSafeStopToIdle();

private:
    // ---------------------- 可根据工程替换为真实子模块 ----------------------
    struct RuntimeState {
        bool lower_alive = false;
        bool imu_ready = false;
        bool posture_ready = false;
        bool grip_confirmed = false;
        bool obstacle_detected = false;
        bool classification_done = false;
        robot_fsm::ObstacleType obstacle_type = robot_fsm::ObstacleType::Unknown;
        robot_fsm::CrossingStrategy crossing_strategy = robot_fsm::CrossingStrategy::None;
        bool at_crossing_position = false;
        bool posture_aligned = false;
        bool crossing_step_done = false;
        bool crossing_complete = false;
        bool post_check_passed = false;
        bool post_check_failed = false;
        double cruise_speed_mps = 0.10;
        double approach_speed_mps = 0.03;
        double damper_cross_speed_mps = 0.02;
        double target_speed_mps = 0.0;
        robot_fsm::DriveMode drive_mode = robot_fsm::DriveMode::Stop;
        std::string joint_limit_callback_event;
    } rt_;

    void updateTelemetryFromLower();
    void updatePerception();
    void updateHealthStatus();
    void buildLowLevelCommand();
    void sendCommandToLower();

    RobotFSMContext fsm_;
};
