#include "driver/mcpwm.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

#define MAX_MOTORS 4
#define SERIAL_BAUD_RATE 460800
#define MOTION_UPDATE_MS 6

// --- 하드웨어 및 모션 설정 ---damuya07@gmail.com
const int servoPins[MAX_MOTORS] = {23, 22, 21, 19};
const int servoMinPulsewidthUs = 500;
const int servoMaxPulsewidthUs = 2500;
const float DEGREES_PER_SECOND = 30.0;
const int DEAD_BAND = 1;

// 안전한 시작을 위한 자세 정의
const int startPose_Assumed[MAX_MOTORS] = {90, 120, 150, 60};
const int homePose_Folded[MAX_MOTORS] = {0, 40, 170, 30};
const int homePose_Observe[MAX_MOTORS] = {90, 140, 168, 30};

// --- FreeRTOS 객체 ---
SemaphoreHandle_t motionMutex;

// --- 모션 제어 로직 ---
namespace MotionController {
    mcpwm_unit_t units[MAX_MOTORS] = {MCPWM_UNIT_0, MCPWM_UNIT_0, MCPWM_UNIT_1, MCPWM_UNIT_1};
    mcpwm_timer_t timers[MAX_MOTORS] = {MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1};
    mcpwm_operator_t operators[MAX_MOTORS] = {MCPWM_OPR_A, MCPWM_OPR_B, MCPWM_OPR_A, MCPWM_OPR_B};
    volatile bool isMoving = false;
    volatile int currentAngles[MAX_MOTORS];
    int startAngles[MAX_MOTORS];
    int targetAngles[MAX_MOTORS];
    int currentStep = 0;
    int totalSteps = 0;

    void initialize() {
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servoPins[0]);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, servoPins[1]);
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, servoPins[2]);
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, servoPins[3]);
        mcpwm_config_t pwm_config;
        pwm_config.frequency = 50;
        pwm_config.cmpr_a = 0; pwm_config.cmpr_b = 0;
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    }

    uint32_t angleToDutyUs(int angle) {
        return map(angle, 0, 180, servoMinPulsewidthUs, servoMaxPulsewidthUs);
    }

    void writeAngle(int motorIndex, int angle) {
        uint32_t duty_us = angleToDutyUs(angle);
        mcpwm_set_duty_in_us(units[motorIndex], timers[motorIndex], operators[motorIndex], duty_us);
        currentAngles[motorIndex] = angle;
    }

    float easeInOutGaussian(float t) {
        t = 2 * t - 1;
        const float sigma = 0.4;
        return (1 + erf(t / (sigma * sqrt(2)))) / 2;
    }

    // [수정 1] 움직일 필요가 없을 때 즉시 ACK를 보내도록 수정
    void startMove(const int newTargets[]) {
        if (xSemaphoreTake(motionMutex, portMAX_DELAY) == pdTRUE) {
            float maxAngleChange = 0;
            bool needsToMove = false;
            for (int i = 0; i < MAX_MOTORS; i++) {
                if (abs(newTargets[i] - currentAngles[i]) > DEAD_BAND) {
                    needsToMove = true;
                }
                startAngles[i] = currentAngles[i];
                targetAngles[i] = constrain(newTargets[i], 0, 180);
                float angleChange = abs(targetAngles[i] - startAngles[i]);
                if (angleChange > maxAngleChange) {
                    maxAngleChange = angleChange;
                }
            }

            if (needsToMove) {
                float totalDurationSeconds = maxAngleChange / DEGREES_PER_SECOND;
                totalSteps = totalDurationSeconds * (1000.0 / MOTION_UPDATE_MS);
                if (totalSteps < 1) totalSteps = 1;
                currentStep = 0;
                isMoving = true;
            } else {
                isMoving = false;
                Serial.println("<D>"); // 이미 도착했으므로 "완료" 신호 즉시 전송
            }
            xSemaphoreGive(motionMutex);
        }
    }

    void updateMotion() {
        if (xSemaphoreTake(motionMutex, portMAX_DELAY) == pdTRUE) {
            if (!isMoving) {
                xSemaphoreGive(motionMutex);
                return;
            }
            float t = (float)currentStep / totalSteps;
            float s = easeInOutGaussian(t);
            for (int i = 0; i < MAX_MOTORS; i++) {
                int angle = startAngles[i] + (targetAngles[i] - startAngles[i]) * s;
                writeAngle(i, angle);
            }
            currentStep++;
            if (currentStep > totalSteps) {
                isMoving = false;
                for (int i = 0; i < MAX_MOTORS; i++) {
                    writeAngle(i, targetAngles[i]);
                }
                Serial.println("<D>"); // 물리적 이동 완료 후 "완료" 신호 전송
            }
            xSemaphoreGive(motionMutex);
        }
    }

    bool isCurrentlyMoving() {
        bool moving_status;
        if (xSemaphoreTake(motionMutex, portMAX_DELAY) == pdTRUE) {
            moving_status = isMoving;
            xSemaphoreGive(motionMutex);
        }
        return moving_status;
    }
}

// --- FreeRTOS 태스크 정의 ---
void motionControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTION_UPDATE_MS);
    for (;;) {
        MotionController::updateMotion();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void communicationTask(void *pvParameters) {
    static char cmdBuffer[64];
    byte bufferIndex = 0;
    for (;;) {
        while (Serial.available() > 0) {
            char incomingChar = Serial.read();
            if (incomingChar == '>') {
                cmdBuffer[bufferIndex] = '\0';
                if (bufferIndex > 0 && strncmp(cmdBuffer, "<M:", 3) == 0) {
                    int angles[MAX_MOTORS];
                    int parsed = sscanf(cmdBuffer + 3, "%d,%d,%d,%d", &angles[0], &angles[1], &angles[2], &angles[3]);
                    
                    // [수정 2] 현재 움직이는 중이 아닐 때만 새 동작을 시작하도록 보호
                    if (parsed == 4 && !MotionController::isCurrentlyMoving()) {
                        MotionController::startMove(angles);
                    }
                }
                bufferIndex = 0;
            } else if (bufferIndex < sizeof(cmdBuffer) - 1) {
                cmdBuffer[bufferIndex++] = incomingChar;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- 메인 프로그램 ---
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    motionMutex = xSemaphoreCreateMutex();
    
    MotionController::initialize();
    for (int i = 0; i < MAX_MOTORS; ++i) {
        MotionController::currentAngles[i] = startPose_Assumed[i];
    }
    
    Serial.println("Homing Step 1: Moving from assumed pose to folded pose...");
    MotionController::startMove(homePose_Folded);
    while (MotionController::isCurrentlyMoving()) {
        MotionController::updateMotion();
        delay(MOTION_UPDATE_MS);
    }
    
    Serial.println("Homing Step 2: Moving from folded pose to observe pose...");
    MotionController::startMove(homePose_Observe);
    while (MotionController::isCurrentlyMoving()) {
        MotionController::updateMotion();
        delay(MOTION_UPDATE_MS);
    }

    Serial.println("Homing Sequence Complete. Starting Tasks...");
    xTaskCreatePinnedToCore(motionControlTask, "MotionTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(communicationTask, "CommTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // FreeRTOS가 모든 것을 처리하므로 비워둡니다.
}
