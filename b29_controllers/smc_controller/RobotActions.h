#pragma once
// ============================================================
// RobotActions.h
// SMC Actions接口 —— RobotContext 需实现的所有方法
// 对应 RobotFSM.sm 中所有 ctxt.xxx() 调用
// ============================================================

#include <string>

// ============================================================
// 枚举：障碍类型
// ============================================================
enum ObstacleType {
    OBS_UNKNOWN    = 0,
    OBS_LINE_CLAMP = 1,   // 线夹
    OBS_DAMPER     = 2,   // 防振锤
};

// ============================================================
// 枚举：越障策略（与障碍类型对应）
// ============================================================
enum CrossingStrategy {
    STRATEGY_NONE       = 0,
    STRATEGY_LINE_CLAMP = 1,
    STRATEGY_DAMPER     = 2,
};

// ============================================================
// 枚举：驱动模式
// ============================================================
enum DriveMode {
    DRIVE_STOP    = 0,
    DRIVE_FORWARD = 1,
};

// ============================================================
// 枚举：告警类型
// ============================================================
enum AlertType {
    ALERT_COMMS_LOSS     = 0,
    ALERT_COMMS_RESTORED = 1,
    ALERT_SAFE_STOP      = 2,
};

// ============================================================
// RobotActions 纯虚接口
// RobotContext 继承并实现全部方法
// ============================================================
class RobotActions {
public:
    virtual ~RobotActions() = default;

    // ----------------------------------------------------------
    // 系统状态查询（Guard条件）
    // ----------------------------------------------------------

    /** 下位机心跳是否存活 */
    virtual bool isLowerAlive() = 0;

    /** IMU是否初始化就绪 */
    virtual bool isImuReady() = 0;

    /** 当前姿态是否已对齐（IMU水平度满足阈值）*/
    virtual bool isPostureReady() = 0;

    /** 线路夹持是否已确认 */
    virtual bool isGripConfirmed() = 0;

    /** 是否检测到障碍（视觉+测距融合判决）*/
    virtual bool isObstacleDetected() = 0;

    /** 障碍分类是否完成 */
    virtual bool isClassificationDone() = 0;

    /** 获取当前障碍类型 */
    virtual ObstacleType getObstacleType() = 0;

    /** 获取当前越障策略 */
    virtual CrossingStrategy getObstacleStrategy() = 0;

    /** 是否已到达越障起始位置 */
    virtual bool isAtCrossingPosition() = 0;

    /** 机身姿态是否对齐完成 */
    virtual bool isPostureAligned() = 0;

    /** 当前越障子步骤是否完成 */
    virtual bool isCrossingStepDone() = 0;

    /** 整个越障序列是否完成 */
    virtual bool isCrossingComplete() = 0;

    /** 越障后自检是否通过 */
    virtual bool isPostCheckPassed() = 0;

    /** 越障后自检是否明确失败 */
    virtual bool isPostCheckFailed() = 0;

    // ----------------------------------------------------------
    // 速度参数获取
    // ----------------------------------------------------------
    virtual double getCruiseSpeed() = 0;     // 巡线速度 m/s
    virtual double getApproachSpeed() = 0;   // 接近速度 m/s
    virtual double getDamperCrossSpeed() = 0;// 防振锤越障速度 m/s

    // ----------------------------------------------------------
    // 运动控制
    // ----------------------------------------------------------

    /** 停止所有电机（驱动轮+关节全部清零） */
    virtual void stopAllMotors() = 0;

    /** 冻结所有关节（保持当前位置，不再接受新目标） */
    virtual void freezeAllJoints() = 0;

    /** 设置驱动轮目标速度 */
    virtual void setTargetSpeed(double speed_mps) = 0;

    /** 设置驱动模式 */
    virtual void setDriveMode(DriveMode mode) = 0;

    /** 发送驱动轮指令到下位机（50Hz调用） */
    virtual void sendDriveCommand() = 0;

    /** 发送关节角度目标到下位机（越障时50Hz调用） */
    virtual void sendJointAngleCommand() = 0;

    // ----------------------------------------------------------
    // 自动行驶控制
    // ----------------------------------------------------------

    /** 启用线路跟踪控制器 */
    virtual void enableLineTracking() = 0;

    /** 禁用线路跟踪控制器 */
    virtual void disableLineTracking() = 0;

    /** 50Hz更新线路跟踪控制量 */
    virtual void updateLineTrackingControl() = 0;

    /** 启用障碍测距传感器 */
    virtual void enableObstacleRanging() = 0;

    /** 禁用障碍测距传感器 */
    virtual void disableObstacleRanging() = 0;

    /** 50Hz更新接近控制量 */
    virtual void updateApproachControl() = 0;

    // ----------------------------------------------------------
    // 越障控制
    // ----------------------------------------------------------

    /** 启动越障动作序列，加载指定策略的预设轨迹 */
    virtual void startCrossingSequence(CrossingStrategy strategy) = 0;

    /** 结束越障序列，清理轨迹规划器 */
    virtual void endCrossingSequence() = 0;

    /** 执行越障下一子步骤，更新关节目标角度 */
    virtual void advanceCrossingStep() = 0;

    /** 启用关节超限监控，注册故障回调 */
    virtual void enableJointMonitor() = 0;

    /** 禁用关节超限监控 */
    virtual void disableJointMonitor() = 0;

    /** 设置关节故障回调事件名（SMC事件名字符串）*/
    virtual void setJointLimitCallback(const std::string& event_name) = 0;

    /** 检查并更新夹持力反馈（防振锤专用）*/
    virtual void checkGripForce() = 0;

    /** 加载越障预备构型（关节预置位）*/
    virtual void loadCrossingPreset(CrossingStrategy strategy) = 0;

    /** 确认越障准备就绪标志 */
    virtual void confirmCrossingReady() = 0;

    // ----------------------------------------------------------
    // 姿态与初始化
    // ----------------------------------------------------------

    /** 初始化自动模式（参数加载、控制器初始化）*/
    virtual void initAutoMode() = 0;

    /** 禁用自动模式，清除所有控制输出 */
    virtual void disableAutoMode() = 0;

    /** 开始初始化序列 */
    virtual void startInitSequence() = 0;

    /** 清除初始化标志位 */
    virtual void clearInitFlags() = 0;

    /** 开始机身姿态对齐（IMU闭环水平校正）*/
    virtual void startPostureAlignment() = 0;

    /** 开始越障后自检 */
    virtual void startPostCrossCheck() = 0;

    /** 清除障碍相关状态 */
    virtual void clearObstacleState() = 0;

    // ----------------------------------------------------------
    // 手动控制
    // ----------------------------------------------------------

    /** 启用手动透传模式 */
    virtual void enableManualPassthrough() = 0;

    /** 禁用手动透传模式 */
    virtual void disableManualPassthrough() = 0;

    // ----------------------------------------------------------
    // 障碍识别
    // ----------------------------------------------------------

    /** 启动障碍识别与分类（视觉+测距融合）*/
    virtual void startObstacleClassification() = 0;

    /** 固化障碍分类结果到状态上下文 */
    virtual void finalizeObstacleClass() = 0;

    /** 根据分类结果设定越障策略 */
    virtual void setObstacleStrategy(CrossingStrategy strategy) = 0;

    /** 记录发现障碍的日志 */
    virtual void logObstacleFound() = 0;

    // ----------------------------------------------------------
    // 通信监控
    // ----------------------------------------------------------

    /** 启动重连计时器（COMMS_LOSS入口）*/
    virtual void startReconnectTimer() = 0;

    /** 停止重连计时器 */
    virtual void stopReconnectTimer() = 0;

    // ----------------------------------------------------------
    // 故障与告警
    // ----------------------------------------------------------

    /** 上报错误信息 */
    virtual void reportError(const std::string& reason) = 0;

    /** 上报通信丢失 */
    virtual void reportCommsLoss() = 0;

    /** 上报紧急停机 */
    virtual void reportEmergencyStop(const std::string& reason) = 0;

    /** 重置所有故障标志位（人工复位时调用）*/
    virtual void resetFaultFlags() = 0;

    /** 发送运营告警 */
    virtual void alertOperator(AlertType type) = 0;

    /** 记录SAFE_STOP入口日志（含时间戳和当前状态快照）*/
    virtual void logSafeStopEntry() = 0;

    /** 通用状态转移日志 */
    virtual void logTransition(const std::string& info) = 0;
};
