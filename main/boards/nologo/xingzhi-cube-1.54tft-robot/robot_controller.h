#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class RobotController {
public:
    static RobotController& GetInstance();

    esp_err_t Start();
    bool SubmitCommand(const char* command, size_t length);
    bool SubmitServoMovement(int servo, int angle, int duration_ms);
    bool SubmitDrive(int left_direction, int right_direction, int duration_ms);
    bool SubmitGreeting();
    bool SubmitLook(int direction);
    bool SubmitStop();

private:
    enum class PatternType {
        NONE,
        GREETING,
        LOOK_LEFT,
        LOOK_RIGHT,
        LOOK_CENTER,
    };

    enum class CommandType {
        SERVO,
        DRIVE,
        PATTERN,
        STOP,
    };

    struct Command {
        CommandType type;
        int servo;
        int angle;
        int left_direction;
        int right_direction;
        int duration_ms;
        PatternType pattern;
    };

    struct ServoMotion {
        int start_angle = 90;
        int target_angle = 90;
        int64_t start_time_us = 0;
        int duration_ms = 0;
        bool active = false;
    };

    RobotController() = default;
    static void ControlTask(void* arg);

    void RegisterMcpTools();
    void RunControlTask();
    void SetServoAngle(int servo, int angle);
    void SetDrive(int left_direction, int right_direction);
    void StopMotors();

    QueueHandle_t command_queue_ = nullptr;
};
