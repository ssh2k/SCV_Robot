#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// 모터 상태 열거형
enum MotorState {
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_TURN_LEFT,
    MOTOR_TURN_RIGHT,
    MOTOR_CURVE_LEFT,
    MOTOR_CURVE_RIGHT,
    MOTOR_CALIBRATING
};

// 모터 방향 열거형
enum MotorDirection {
    MOTOR_FORWARD_DIR,
    MOTOR_BACKWARD_DIR
};

// 모터 제어 클래스 (Pololu TB9051FTG 3핀 제어 방식)
class MotorControl {
public:
    // 생성자: Pololu TB9051FTG 3핀 제어 방식
    // left_in1, left_in2, left_pwm, right_in1, right_in2, right_pwm
    MotorControl(int left_in1_pin, int left_in2_pin, int left_pwm_pin, 
                 int right_in1_pin, int right_in2_pin, int right_pwm_pin);
    
    // 소멸자
    ~MotorControl();

    // 초기화 및 설정
    void begin();
    void setMaxSpeed(int maxSpeed);
    void setMinSpeed(int minSpeed);
    void enableLogging(bool enable);
    
    // 기본 이동 명령
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stop();
    
    // 고급 이동 명령 (곡선 이동)
    void curveLeft(int leftSpeed, int rightSpeed);
    void curveRight(int leftSpeed, int rightSpeed);
    
    // 개별 모터 제어
    void setLeftMotor(int speed, MotorDirection direction);
    void setRightMotor(int speed, MotorDirection direction);
    void setLeftMotorSpeed(int speed);
    void setRightMotorSpeed(int speed);
    
    // 속도 제어
    void setSpeed(int speed);
    int getCurrentSpeed() const;
    int getLeftSpeed() const;
    int getRightSpeed() const;
    
    // 상태 확인
    MotorState getCurrentState() const;
    bool isMoving() const;
    bool isCalibrating() const;
    
    // 안전 기능
    void emergencyStop();
    void softStop(); // 비블로킹 부드러운 정지
    void softStopAsync(); // 비동기 부드러운 정지
    
    // 캘리브레이션
    void startCalibration();
    void stopCalibration();
    bool isCalibrated() const;
    
    // 유틸리티
    void reset();
    void testMotors();
    void printStatus();

private:
    // Pololu TB9051FTG 3핀 설정
    int _left_in1_pin;
    int _left_in2_pin;
    int _left_pwm_pin;
    int _right_in1_pin;
    int _right_in2_pin;
    int _right_pwm_pin;
    
    // 현재 상태
    MotorState _currentState;
    int _currentSpeed;
    int _leftSpeed;
    int _rightSpeed;
    
    // 설정값
    int _maxSpeed;
    int _minSpeed;
    bool _loggingEnabled;
    bool _isCalibrated;
    
    // 비동기 정지 관련
    bool _softStopInProgress;
    unsigned long _softStopStartTime;
    int _softStopInitialLeft;
    int _softStopInitialRight;
    
    // 내부 헬퍼 함수
    void _setMotorPins(int leftSpeed, int rightSpeed);
    void _updateState(MotorState newState);
    int _constrainSpeed(int speed);
    void _logMotorAction(const char* action, int leftSpeed, int rightSpeed);
    void _updateSoftStop();
    bool _validateSpeed(int speed);
    // Pololu TB9051FTG 3핀 제어 함수
    void _setMotorDirection(int in1Pin, int in2Pin, int pwmPin, int speed);
};

#endif