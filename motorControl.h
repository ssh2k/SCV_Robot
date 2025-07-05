#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Arduino.h"

class MotorControl {
public:
    // 생성자: 모터 제어에 필요한 핀 번호를 받아서 초기화합니다.
    MotorControl(int left_pwm_pin, int left_dir_pin, int right_pwm_pin, int right_dir_pin);

    // 모터 핀을 OUTPUT으로 설정하는 초기화 함수
    void begin();

    // 전진
    void forward(int speed);

    // 후진
    void backward(int speed);

    // 좌회전 (제자리 회전)
    void turnLeft(int speed);

    // 우회전 (제자리 회전)
    void turnRight(int speed);

    // 정지
    void stop();

private:
    // 각 모터의 PWM(속도)과 DIR(방향) 핀 번호를 저장할 변수
    int _left_pwm_pin;
    int _left_dir_pin;
    int _right_pwm_pin;
    int _right_dir_pin;
};

#endif
