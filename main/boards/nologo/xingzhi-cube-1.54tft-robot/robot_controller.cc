#include "robot_controller.h"

#include <algorithm>
#include <cstring>

#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "mcp_server.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "RobotController"

namespace {

constexpr char kBleDeviceName[] = "Xiaozhi-Robot";
constexpr int kServoMinAngle = 45;
constexpr int kServoMaxAngle = 135;
constexpr int kServoInitialAngle = 90;
constexpr int kMotorPulseMs = 400;
constexpr int kServoMoveDurationMs = 700;
constexpr int kControlTickMs = 20;
constexpr int kPatternMoveDurationMs = 450;
constexpr int kPatternStepIntervalMs = 550;
constexpr int kMinDriveDurationMs = 100;
constexpr int kMaxDriveDurationMs = 1000;
constexpr int kMinServoDurationMs = 300;
constexpr int kMaxServoDurationMs = 2000;
constexpr int kGreetingAngles[][2] = {
    {75, 105}, {105, 75}, {75, 105}, {105, 75}, {75, 105}, {105, 75}, {90, 90},
};
constexpr int kGreetingStepCount = sizeof(kGreetingAngles) / sizeof(kGreetingAngles[0]);

constexpr gpio_num_t kServoPins[] = {
    ROBOT_SERVO_LEFT_ARM_GPIO,
    ROBOT_SERVO_RIGHT_ARM_GPIO,
    ROBOT_SERVO_HEAD_GPIO,
};

constexpr gpio_num_t kMotorPins[] = {
    ROBOT_MOTOR_IN1_GPIO,
    ROBOT_MOTOR_IN2_GPIO,
    ROBOT_MOTOR_IN3_GPIO,
    ROBOT_MOTOR_IN4_GPIO,
};

constexpr ledc_channel_t kServoChannels[] = {
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

constexpr uint32_t kServoPwmResolution = 1 << LEDC_TIMER_13_BIT;
constexpr uint32_t kServoPwmPeriodUs = 20000;

static ble_uuid128_t kUartServiceUuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static ble_uuid128_t kUartRxUuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                                                    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static ble_uuid128_t kUartTxUuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                                                    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_uart_rx_handle;
static uint16_t s_uart_tx_handle;

static int GattAccess(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                      void* arg);
static int GapEvent(struct ble_gap_event* event, void* arg);
static void StartAdvertising();
static void NimbleHostTask(void* arg);
static void OnNimbleSync();

static const struct ble_gatt_chr_def kUartCharacteristics[] = {
    {
        .uuid = &kUartRxUuid.u,
        .access_cb = GattAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = &s_uart_rx_handle,
    },
    {
        .uuid = &kUartTxUuid.u,
        .access_cb = GattAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_uart_tx_handle,
    },
    {},
};

static const struct ble_gatt_svc_def kUartServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kUartServiceUuid.u,
        .includes = nullptr,
        .characteristics = kUartCharacteristics,
    },
    {},
};

uint32_t ServoDutyForAngle(int angle) {
    const uint32_t pulse_us = 500 + (2000 * angle / 180);
    return kServoPwmResolution * pulse_us / kServoPwmPeriodUs;
}

int GattAccess(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
               void* arg) {
    (void)conn_handle;
    (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && attr_handle == s_uart_tx_handle) {
        static constexpr char kReady[] = "ready";
        return os_mbuf_append(ctxt->om, kReady, sizeof(kReady) - 1) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || attr_handle != s_uart_rx_handle) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const size_t length = OS_MBUF_PKTLEN(ctxt->om);
    char command[24];
    if (length == 0 || length >= sizeof(command) ||
        os_mbuf_copydata(ctxt->om, 0, length, command) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    command[length] = '\0';
    size_t command_length = length;
    while (command_length > 0 &&
           (command[command_length - 1] == '\r' || command[command_length - 1] == '\n' ||
            command[command_length - 1] == ' ')) {
        command[--command_length] = '\0';
    }

    if (!RobotController::GetInstance().SubmitCommand(command, command_length)) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

int GapEvent(struct ble_gap_event* event, void* arg) {
    (void)arg;
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "BLE client connected");
            } else {
                StartAdvertising();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE client disconnected");
            RobotController::GetInstance().SubmitCommand("M:S", 3);
            StartAdvertising();
            break;
        default:
            break;
    }
    return 0;
}

void StartAdvertising() {
    uint8_t address_type;
    int result = ble_hs_id_infer_auto(0, &address_type);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot determine BLE address type: %d", result);
        return;
    }

    struct ble_hs_adv_fields advertising_fields = {};
    advertising_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advertising_fields.uuids128 = &kUartServiceUuid;
    advertising_fields.num_uuids128 = 1;
    advertising_fields.uuids128_is_complete = 1;

    result = ble_gap_adv_set_fields(&advertising_fields);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot set BLE advertising data: %d", result);
        return;
    }

    struct ble_hs_adv_fields scan_response_fields = {};
    scan_response_fields.name = reinterpret_cast<const uint8_t*>(kBleDeviceName);
    scan_response_fields.name_len = sizeof(kBleDeviceName) - 1;
    scan_response_fields.name_is_complete = 1;
    result = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot set BLE scan response data: %d", result);
        return;
    }

    struct ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    result = ble_gap_adv_start(address_type, nullptr, BLE_HS_FOREVER, &params, GapEvent, nullptr);
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot start BLE advertising: %d", result);
    } else {
        ESP_LOGI(TAG, "BLE UART advertising as %s", kBleDeviceName);
    }
}

void OnNimbleSync() { StartAdvertising(); }

void OnNimbleReset(int reason) { ESP_LOGE(TAG, "NimBLE host reset: %d", reason); }

void NimbleHostTask(void* arg) {
    (void)arg;
    nimble_port_run();
    vTaskDelete(nullptr);
}

}  // namespace

RobotController& RobotController::GetInstance() {
    static RobotController instance;
    return instance;
}

esp_err_t RobotController::Start() {
    if (command_queue_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t motor_config = {};
    motor_config.mode = GPIO_MODE_OUTPUT;
    motor_config.pin_bit_mask = 0;
    for (gpio_num_t pin : kMotorPins) {
        motor_config.pin_bit_mask |= 1ULL << pin;
    }
    motor_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&motor_config), TAG, "Configure motor GPIOs");
    StopMotors();

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = LEDC_TIMER_13_BIT;
    timer_config.timer_num = LEDC_TIMER_1;
    timer_config.freq_hz = 50;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Configure servo PWM");

    for (int i = 0; i < 3; ++i) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num = kServoPins[i];
        channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_config.channel = kServoChannels[i];
        channel_config.timer_sel = LEDC_TIMER_1;
        channel_config.duty = ServoDutyForAngle(kServoInitialAngle);
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG,
                            "Configure servo PWM channel");
    }

    command_queue_ = xQueueCreate(8, sizeof(Command));
    if (command_queue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(ControlTask, "robot_control", 3072, this, 5, nullptr) != pdPASS) {
        vQueueDelete(command_queue_);
        command_queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    RegisterMcpTools();
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "Initialize NimBLE host");

    ble_hs_cfg.reset_cb = OnNimbleReset;
    ble_hs_cfg.sync_cb = OnNimbleSync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_RETURN_ON_ERROR(ble_svc_gap_device_name_set(kBleDeviceName), TAG, "Set BLE device name");

    int result = ble_gatts_count_cfg(kUartServices);
    if (result == 0) {
        result = ble_gatts_add_svcs(kUartServices);
    }
    if (result != 0) {
        ESP_LOGE(TAG, "Cannot register BLE UART service: %d", result);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(NimbleHostTask);
    return ESP_OK;
}

void RobotController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool(
        "self.robot.drive",
        "Mover las ruedas solo durante un pulso corto; siempre se detienen automáticamente "
        "después de 100-1000 ms. left/right: 1 adelante, -1 atrás, 0 quieta. Avanzar=(1,1), "
        "retroceder=(-1,-1), girar a la izquierda=(-1,1), girar a la derecha=(1,-1). "
        "Para 'comienza a andar' o 'muévete', avanza con pulsos breves y vuelve a escuchar "
        "entre pulsos; si la persona dice 'detente', llama self.robot.stop inmediatamente. "
        "No supongas que una duración equivale a una distancia exacta.",
        PropertyList({Property("left", kPropertyTypeInteger, 0, -1, 1),
                      Property("right", kPropertyTypeInteger, 0, -1, 1),
                      Property("duration_ms", kPropertyTypeInteger, 400, kMinDriveDurationMs,
                               kMaxDriveDurationMs)}),
        [this](const PropertyList& properties) -> ToolResult {
            const int left = properties["left"].value<int>();
            const int right = properties["right"].value<int>();
            const int duration_ms = properties["duration_ms"].value<int>();
            if (!SubmitDrive(left, right, duration_ms)) {
                return std::unexpected("Robot drive command queue is full or invalid");
            }
            return ReturnValue{true};
        });

    mcp_server.AddTool(
        "self.robot.move_servo",
        "Mover suavemente un servo. servo: 0=brazo izquierdo, 1=brazo derecho, "
        "2=cabeza; angle: 45-135 grados; duration_ms: 300-2000.",
        PropertyList({Property("servo", kPropertyTypeInteger, 0, 0, 2),
                      Property("angle", kPropertyTypeInteger, 90, kServoMinAngle, kServoMaxAngle),
                      Property("duration_ms", kPropertyTypeInteger, 700, kMinServoDurationMs,
                               kMaxServoDurationMs)}),
        [this](const PropertyList& properties) -> ToolResult {
            const int servo = properties["servo"].value<int>();
            const int angle = properties["angle"].value<int>();
            const int duration_ms = properties["duration_ms"].value<int>();
            if (!SubmitServoMovement(servo, angle, duration_ms)) {
                return std::unexpected("Robot servo command queue is full or invalid");
            }
            return ReturnValue{true};
        });

    mcp_server.AddTool("self.robot.stop",
                       "Detener inmediatamente ambas ruedas y descartar movimientos pendientes. "
                       "Llamar en cuanto la persona diga 'detente', 'para', 'basta' o 'quieto'.",
                       PropertyList(), [this](const PropertyList&) -> ToolResult {
                           if (!SubmitStop()) {
                               return std::unexpected("Unable to queue robot stop command");
                           }
                           return ReturnValue{true};
                       });

    mcp_server.AddTool(
        "self.robot.greet",
        "Hacer un saludo visible moviendo suavemente ambos brazos en alternancia tres veces y "
        "volver al centro. Usar cuando la persona diga saluda, hola o despídete.",
        PropertyList(), [this](const PropertyList&) -> ToolResult {
            if (!SubmitGreeting()) {
                return std::unexpected("Robot greeting command queue is full");
            }
            return ReturnValue{true};
        });

    mcp_server.AddTool(
        "self.robot.look",
        "Girar suavemente la cabeza hacia la dirección indicada y volver al centro: "
        "direction=-1 izquierda, 1 derecha, 0 centro.",
        PropertyList({Property("direction", kPropertyTypeInteger, 0, -1, 1)}),
        [this](const PropertyList& properties) -> ToolResult {
            if (!SubmitLook(properties["direction"].value<int>())) {
                return std::unexpected("Robot look command queue is full or invalid");
            }
            return ReturnValue{true};
        });
}

bool RobotController::SubmitCommand(const char* command, size_t length) {
    if (command_queue_ == nullptr || command == nullptr) {
        return false;
    }

    Command parsed = {};
    if (length >= 6 && length <= 7 && command[0] == 'S' && command[1] == ':' && command[3] == ':') {
        switch (command[2]) {
            case 'L':
                parsed.servo = 0;
                break;
            case 'R':
                parsed.servo = 1;
                break;
            case 'H':
                parsed.servo = 2;
                break;
            default:
                return false;
        }

        const char* angle_text = command + 4;
        const size_t angle_length = length - 4;
        int angle = 0;
        for (size_t i = 0; i < angle_length; ++i) {
            if (angle_text[i] < '0' || angle_text[i] > '9') {
                return false;
            }
            angle = angle * 10 + angle_text[i] - '0';
        }
        if (angle_length < 2 || angle_length > 3) {
            return false;
        }
        if (angle < kServoMinAngle || angle > kServoMaxAngle) {
            return false;
        }
        parsed.type = CommandType::SERVO;
        parsed.angle = angle;
        parsed.duration_ms = kServoMoveDurationMs;
    } else if (length == 3 && command[1] == ':') {
        if (command[0] == 'M' && command[2] == 'S') {
            return SubmitStop();
        }
        if (command[2] != 'F' && command[2] != 'B') {
            return false;
        }
        const int direction = command[2] == 'F' ? 1 : -1;
        parsed.type = CommandType::DRIVE;
        parsed.duration_ms = kMotorPulseMs;
        switch (command[0]) {
            case 'A':
                parsed.left_direction = direction;
                break;
            case 'B':
                parsed.right_direction = direction;
                break;
            case 'M':
                parsed.left_direction = direction;
                parsed.right_direction = direction;
                break;
            default:
                return false;
        }
    } else {
        return false;
    }

    if (xQueueSend(command_queue_, &parsed, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Robot command queue is full");
        return false;
    }
    return true;
}

bool RobotController::SubmitServoMovement(int servo, int angle, int duration_ms) {
    if (command_queue_ == nullptr || servo < 0 || servo >= 3 || angle < kServoMinAngle ||
        angle > kServoMaxAngle || duration_ms < kMinServoDurationMs ||
        duration_ms > kMaxServoDurationMs) {
        return false;
    }

    const Command command = {
        .type = CommandType::SERVO,
        .servo = servo,
        .angle = angle,
        .duration_ms = duration_ms,
    };
    if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Robot command queue is full");
        return false;
    }
    return true;
}

bool RobotController::SubmitDrive(int left_direction, int right_direction, int duration_ms) {
    if (command_queue_ == nullptr || left_direction < -1 || left_direction > 1 ||
        right_direction < -1 || right_direction > 1 || duration_ms < kMinDriveDurationMs ||
        duration_ms > kMaxDriveDurationMs) {
        return false;
    }
    if (left_direction == 0 && right_direction == 0) {
        return SubmitStop();
    }

    const Command command = {
        .type = CommandType::DRIVE,
        .left_direction = left_direction,
        .right_direction = right_direction,
        .duration_ms = duration_ms,
    };
    if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Robot command queue is full");
        return false;
    }
    return true;
}

bool RobotController::SubmitGreeting() {
    if (command_queue_ == nullptr) {
        return false;
    }

    const Command command = {
        .type = CommandType::PATTERN,
        .pattern = PatternType::GREETING,
    };
    if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Robot command queue is full");
        return false;
    }
    return true;
}

bool RobotController::SubmitLook(int direction) {
    if (command_queue_ == nullptr || direction < -1 || direction > 1) {
        return false;
    }

    const PatternType pattern = direction < 0   ? PatternType::LOOK_LEFT
                                : direction > 0 ? PatternType::LOOK_RIGHT
                                                : PatternType::LOOK_CENTER;
    const Command command = {
        .type = CommandType::PATTERN,
        .pattern = pattern,
    };
    if (xQueueSend(command_queue_, &command, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Robot command queue is full");
        return false;
    }
    return true;
}

bool RobotController::SubmitStop() {
    if (command_queue_ == nullptr) {
        return false;
    }
    xQueueReset(command_queue_);
    const Command command = {.type = CommandType::STOP};
    if (xQueueSendToFront(command_queue_, &command, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Unable to queue robot stop command");
        return false;
    }
    return true;
}

void RobotController::ControlTask(void* arg) {
    static_cast<RobotController*>(arg)->RunControlTask();
}

void RobotController::RunControlTask() {
    Command command;
    ServoMotion servo_motions[3] = {};
    int servo_angles[3] = {kServoInitialAngle, kServoInitialAngle, kServoInitialAngle};
    int64_t motor_stop_at = 0;
    PatternType active_pattern = PatternType::NONE;
    int pattern_stage = 0;
    int64_t pattern_next_step_at = 0;

    while (true) {
        TickType_t wait = pdMS_TO_TICKS(kControlTickMs);
        if (motor_stop_at != 0) {
            const int64_t remaining_us = motor_stop_at - esp_timer_get_time();
            if (remaining_us <= 0) {
                wait = 0;
            } else {
                wait = std::min(wait, pdMS_TO_TICKS((remaining_us + 999) / 1000));
            }
        }

        if (xQueueReceive(command_queue_, &command, wait) == pdTRUE) {
            active_pattern = PatternType::NONE;
            if (command.type == CommandType::SERVO) {
                ServoMotion& motion = servo_motions[command.servo];
                motion.start_angle = servo_angles[command.servo];
                motion.target_angle = command.angle;
                motion.start_time_us = esp_timer_get_time();
                motion.duration_ms = command.duration_ms;
                motion.active = true;
            } else if (command.type == CommandType::STOP) {
                StopMotors();
                motor_stop_at = 0;
            } else if (command.type == CommandType::PATTERN) {
                active_pattern = command.pattern;
                pattern_stage = 0;
                pattern_next_step_at = 0;
            } else {
                SetDrive(command.left_direction, command.right_direction);
                motor_stop_at =
                    esp_timer_get_time() + static_cast<int64_t>(command.duration_ms) * 1000;
            }
        }

        const int64_t now_us = esp_timer_get_time();
        if (active_pattern != PatternType::NONE && now_us >= pattern_next_step_at) {
            if (active_pattern == PatternType::GREETING) {
                if (pattern_stage < kGreetingStepCount) {
                    for (int servo = 0; servo < 2; ++servo) {
                        ServoMotion& motion = servo_motions[servo];
                        motion.start_angle = servo_angles[servo];
                        motion.target_angle = kGreetingAngles[pattern_stage][servo];
                        motion.start_time_us = now_us;
                        motion.duration_ms = kPatternMoveDurationMs;
                        motion.active = true;
                    }
                    ++pattern_stage;
                    pattern_next_step_at =
                        now_us + static_cast<int64_t>(kPatternStepIntervalMs) * 1000;
                } else {
                    active_pattern = PatternType::NONE;
                }
            } else {
                if (pattern_stage < 2) {
                    const int target_angle =
                        pattern_stage == 0
                            ? (active_pattern == PatternType::LOOK_LEFT ? 70 : 110)
                            : kServoInitialAngle;
                    ServoMotion& motion = servo_motions[2];
                    motion.start_angle = servo_angles[2];
                    motion.target_angle = target_angle;
                    motion.start_time_us = now_us;
                    motion.duration_ms = kPatternMoveDurationMs;
                    motion.active = true;
                    ++pattern_stage;
                    pattern_next_step_at =
                        now_us + static_cast<int64_t>(kPatternStepIntervalMs) * 1000;
                } else {
                    active_pattern = PatternType::NONE;
                }
            }
        }

        for (int servo = 0; servo < 3; ++servo) {
            ServoMotion& motion = servo_motions[servo];
            if (!motion.active) {
                continue;
            }

            const int64_t elapsed_us = now_us - motion.start_time_us;
            const int64_t duration_us = static_cast<int64_t>(motion.duration_ms) * 1000;
            if (elapsed_us >= duration_us) {
                servo_angles[servo] = motion.target_angle;
                motion.active = false;
            } else {
                const int delta = motion.target_angle - motion.start_angle;
                servo_angles[servo] =
                    motion.start_angle + static_cast<int>(delta * elapsed_us / duration_us);
            }
            SetServoAngle(servo, servo_angles[servo]);
        }

        if (motor_stop_at != 0 && esp_timer_get_time() >= motor_stop_at) {
            StopMotors();
            motor_stop_at = 0;
        }
    }
}

void RobotController::SetServoAngle(int servo, int angle) {
    const uint32_t duty = ServoDutyForAngle(angle);
    const auto channel = kServoChannels[servo];
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void RobotController::SetDrive(int left_direction, int right_direction) {
    StopMotors();
    const int directions[] = {left_direction, -right_direction};
    for (int motor = 0; motor < 2; ++motor) {
        if (directions[motor] == 0) {
            continue;
        }
        const int first_pin = motor * 2;
        ESP_ERROR_CHECK(gpio_set_level(kMotorPins[first_pin], directions[motor] > 0 ? 1 : 0));
        ESP_ERROR_CHECK(gpio_set_level(kMotorPins[first_pin + 1], directions[motor] > 0 ? 0 : 1));
    }
}

void RobotController::StopMotors() {
    for (gpio_num_t pin : kMotorPins) {
        ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    }
}
