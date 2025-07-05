#include "MotorControl.h"

// 생성자 구현
MotorControl::MotorControl(int left_pwm_pin, int left_dir_pin, int right_pwm_pin, int right_dir_pin) {
    _left_pwm_pin = left_pwm_pin;
    _left_dir_pin = left_dir_pin;
    _right_pwm_pin = right_pwm_pin;
    _right_dir_pin = right_dir_pin;
}

// 초기화 함수 구현
void MotorControl::begin() {
    pinMode(_left_pwm_pin, OUTPUT);
    pinMode(_left_dir_pin, OUTPUT);
    pinMode(_right_pwm_pin, OUTPUT);
    pinMode(_right_dir_pin, OUTPUT);
}

// 전진 함수 구현
void MotorControl::forward(int speed) {
    // 모터 배선에 따라 HIGH/LOW가 반대일 수 있습니다.
    // 만약 뒤로 간다면 아래 두 줄의 HIGH/LOW를 서로 바꿔주세요.
    digitalWrite(_left_dir_pin, HIGH);
    digitalWrite(_right_dir_pin, HIGH);

    analogWrite(_left_pwm_pin, speed);
    analogWrite(_right_pwm_pin, speed);
}

// 후진 함수 구현
void MotorControl::backward(int speed) {
    digitalWrite(_left_dir_pin, LOW);
    digitalWrite(_right_dir_pin, LOW);

    analogWrite(_left_pwm_pin, speed);
    analogWrite(_right_pwm_pin, speed);
}

// 좌회전 함수 구현 (오른쪽 바퀴는 앞으로, 왼쪽 바퀴는 뒤로)
void MotorControl::turnLeft(int speed) {
    digitalWrite(_left_dir_pin, LOW);   // 왼쪽 바퀴 뒤로
    digitalWrite(_right_dir_pin, HIGH); // 오른쪽 바퀴 앞으로

    analogWrite(_left_pwm_pin, speed);
    analogWrite(_right_pwm_pin, speed);
}

// 우회전 함수 구현 (왼쪽 바퀴는 앞으로, 오른쪽 바퀴는 뒤로)
void MotorControl::turnRight(int speed) {
    digitalWrite(_left_dir_pin, HIGH); // 왼쪽 바퀴 앞으로
    digitalWrite(_right_dir_pin, LOW);  // 오른쪽 바퀴 뒤로

    analogWrite(_left_pwm_pin, speed);
    analogWrite(_right_pwm_pin, speed);
}

// 정지 함수 구현
void MotorControl::stop() {
    analogWrite(_left_pwm_pin, 0);
    analogWrite(_right_pwm_pin, 0);
}
