#include "MotorControl.h"

// --- 핀 설정 ---
// 아두이노 R4 보드의 실제 연결된 핀 번호로 수정하세요.
// PWM 기능이 있는 핀(~)을 PWM 핀으로 사용해야 합니다. (예: 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PWM_PIN  9
#define LEFT_MOTOR_DIR_PIN  8
#define RIGHT_MOTOR_PWM_PIN 10
#define RIGHT_MOTOR_DIR_PIN 11

// 위에서 설정한 핀으로 MotorControl 객체를 생성합니다.
MotorControl motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);

void setup() {
    Serial.begin(9600);
    while (!Serial); // 시리얼 포트가 열릴 때까지 대기

    Serial.println("Motor Control Test Start!");
    motor.begin(); // 모터 핀 초기화
}

void loop() {
    int test_speed = 200; // 테스트 속도 (0 ~ 255)

    Serial.println("Forward...");
    motor.forward(test_speed);
    delay(2000); // 2초간 실행
    motor.stop();
    delay(1000); // 1초간 정지

    Serial.println("Backward...");
    motor.backward(test_speed);
    delay(2000);
    motor.stop();
    delay(1000);

    Serial.println("Turn Left...");
    motor.turnLeft(test_speed);
    delay(1000); // 1초간 회전
    motor.stop();
    delay(1000);

    Serial.println("Turn Right...");
    motor.turnRight(test_speed);
    delay(1000);
    motor.stop();
    delay(1000);
}
