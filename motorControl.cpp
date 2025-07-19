#include "motorControl.h"
#include "utils.h"

MotorControl::MotorControl(int left_in1_pin, int left_in2_pin, int left_pwm_pin, 
                         int right_in1_pin, int right_in2_pin, int right_pwm_pin) {
    _left_in1_pin = left_in1_pin;
    _left_in2_pin = left_in2_pin;
    _left_pwm_pin = left_pwm_pin;
    _right_in1_pin = right_in1_pin;
    _right_in2_pin = right_in2_pin;
    _right_pwm_pin = right_pwm_pin;
    
    // 상태 초기화
    _currentState = MOTOR_STOP;
    _currentSpeed = 0;
    _leftSpeed = 0;
    _rightSpeed = 0;
    
    // 설정값 초기화
    _maxSpeed = 255;
    _minSpeed = 0;
    _loggingEnabled = true;
    _isCalibrated = false;
    
    // 비동기 정지 관련
    _softStopInProgress = false;
    _softStopStartTime = 0;
    _softStopInitialLeft = 0;
    _softStopInitialRight = 0;
}

MotorControl::~MotorControl() {
    // 안전을 위해 정지
    emergencyStop();
}

void MotorControl::begin() {
    // Pololu TB9051FTG 3핀 모드 설정
    pinMode(_left_in1_pin, OUTPUT);
    pinMode(_left_in2_pin, OUTPUT);
    pinMode(_left_pwm_pin, OUTPUT);
    pinMode(_right_in1_pin, OUTPUT);
    pinMode(_right_in2_pin, OUTPUT);
    pinMode(_right_pwm_pin, OUTPUT);
    
    // 초기 상태 설정
    emergencyStop();
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Pololu TB9051FTG motor control system initialized");
    }
}

void MotorControl::setMaxSpeed(int maxSpeed) {
    _maxSpeed = constrainSpeed(maxSpeed, 0, 255);
    // 로깅 제거 - 설정값은 필요시에만 출력
}

void MotorControl::setMinSpeed(int minSpeed) {
    _minSpeed = constrainSpeed(minSpeed, 0, _maxSpeed);
    // 로깅 제거 - 설정값은 필요시에만 출력
}

void MotorControl::enableLogging(bool enable) {
    _loggingEnabled = enable;
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Logging enabled");
    }
}

void MotorControl::forward(int speed) {
    if (!_validateSpeed(speed)) return;
    
    _setMotorPins(speed, speed);
    _updateState(MOTOR_FORWARD);
    _logMotorAction("FORWARD", speed, speed);
}

void MotorControl::backward(int speed) {
    if (!_validateSpeed(speed)) return;
    
    _setMotorPins(-speed, -speed);
    _updateState(MOTOR_BACKWARD);
    _logMotorAction("BACKWARD", -speed, -speed);
}

void MotorControl::turnLeft(int speed) {
    if (!_validateSpeed(speed)) return;
    
    _setMotorPins(-speed, speed);
    _updateState(MOTOR_TURN_LEFT);
    _logMotorAction("TURN LEFT", -speed, speed);
}

void MotorControl::turnRight(int speed) {
    if (!_validateSpeed(speed)) return;
    
    _setMotorPins(speed, -speed);
    _updateState(MOTOR_TURN_RIGHT);
    _logMotorAction("TURN RIGHT", speed, -speed);
}

void MotorControl::curveLeft(int leftSpeed, int rightSpeed) {
    if (!_validateSpeed(leftSpeed) || !_validateSpeed(rightSpeed)) return;
    
    _setMotorPins(leftSpeed, rightSpeed);
    _updateState(MOTOR_CURVE_LEFT);
    _logMotorAction("CURVE LEFT", leftSpeed, rightSpeed);
}

void MotorControl::curveRight(int leftSpeed, int rightSpeed) {
    if (!_validateSpeed(leftSpeed) || !_validateSpeed(rightSpeed)) return;
    
    _setMotorPins(leftSpeed, rightSpeed);
    _updateState(MOTOR_CURVE_RIGHT);
    _logMotorAction("CURVE RIGHT", leftSpeed, rightSpeed);
}

void MotorControl::setLeftMotor(int speed, MotorDirection direction) {
    if (!_validateSpeed(speed)) return;
    
    if (direction == MOTOR_BACKWARD_DIR) {
        speed = -speed;
    }
    
    _leftSpeed = speed;
    _setMotorDirection(_left_in1_pin, _left_in2_pin, _left_pwm_pin, speed);
    
    // 개별 모터 설정 로그 제거
}

void MotorControl::setRightMotor(int speed, MotorDirection direction) {
    if (!_validateSpeed(speed)) return;
    
    if (direction == MOTOR_BACKWARD_DIR) {
        speed = -speed;
    }
    
    _rightSpeed = speed;
    _setMotorDirection(_right_in1_pin, _right_in2_pin, _right_pwm_pin, speed);
    
    // 개별 모터 설정 로그 제거
}

void MotorControl::setLeftMotorSpeed(int speed) {
    if (!_validateSpeed(abs(speed))) return;
    
    _leftSpeed = speed;
    _setMotorDirection(_left_in1_pin, _left_in2_pin, _left_pwm_pin, speed);
}

void MotorControl::setRightMotorSpeed(int speed) {
    if (!_validateSpeed(abs(speed))) return;
    
    _rightSpeed = speed;
    _setMotorDirection(_right_in1_pin, _right_in2_pin, _right_pwm_pin, speed);
}

void MotorControl::stop() {
    _setMotorPins(0, 0);
    _updateState(MOTOR_STOP);
    _softStopInProgress = false;
    
    // 정지 로그 제거
}

void MotorControl::emergencyStop() {
    // 즉시 정지 - 모든 핀을 LOW로 설정
    digitalWrite(_left_in1_pin, LOW);
    digitalWrite(_left_in2_pin, LOW);
    analogWrite(_left_pwm_pin, 0);
    digitalWrite(_right_in1_pin, LOW);
    digitalWrite(_right_in2_pin, LOW);
    analogWrite(_right_pwm_pin, 0);
    
    _currentState = MOTOR_STOP;
    _currentSpeed = 0;
    _leftSpeed = 0;
    _rightSpeed = 0;
    _softStopInProgress = false;
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] EMERGENCY STOP ACTIVATED");
    }
}

void MotorControl::softStop() {
    // 비블로킹 부드러운 정지
    _softStopInProgress = true;
    _softStopStartTime = millis();
    _softStopInitialLeft = _leftSpeed;
    _softStopInitialRight = _rightSpeed;
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Soft stop initiated");
    }
}

void MotorControl::softStopAsync() {
    // 비동기 부드러운 정지 업데이트
    if (_softStopInProgress) {
        _updateSoftStop();
    }
}

void MotorControl::startCalibration() {
    _updateState(MOTOR_CALIBRATING);
    _isCalibrated = false;
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Motor calibration started");
    }
}

void MotorControl::stopCalibration() {
    if (_currentState == MOTOR_CALIBRATING) {
        stop();
        _isCalibrated = true;
        
        if (_loggingEnabled) {
            Serial.println("[MotorControl] Motor calibration completed");
        }
    }
}

bool MotorControl::isCalibrated() const {
    return _isCalibrated;
}

void MotorControl::reset() {
    emergencyStop();
    _isCalibrated = false;
    _softStopInProgress = false;
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Motor control system reset");
    }
}

void MotorControl::testMotors() {
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Starting motor test sequence");
    }
    
    // 왼쪽 모터 테스트
    setLeftMotor(100, MOTOR_FORWARD_DIR);
    delay(500);
    setLeftMotor(0, MOTOR_FORWARD_DIR);
    delay(200);
    
    // 오른쪽 모터 테스트
    setRightMotor(100, MOTOR_FORWARD_DIR);
    delay(500);
    setRightMotor(0, MOTOR_FORWARD_DIR);
    delay(200);
    
    if (_loggingEnabled) {
        Serial.println("[MotorControl] Motor test sequence completed");
    }
}

void MotorControl::printStatus() {
    Serial.println("=== Motor Control Status ===");
    Serial.print("State: ");
    Serial.println(_currentState);
    Serial.print("Current Speed: ");
    Serial.println(_currentSpeed);
    Serial.print("Left Speed: ");
    Serial.println(_leftSpeed);
    Serial.print("Right Speed: ");
    Serial.println(_rightSpeed);
    Serial.print("Max Speed: ");
    Serial.println(_maxSpeed);
    Serial.print("Min Speed: ");
    Serial.println(_minSpeed);
    Serial.print("Calibrated: ");
    Serial.println(_isCalibrated ? "Yes" : "No");
    Serial.print("Soft Stop: ");
    Serial.println(_softStopInProgress ? "In Progress" : "Idle");
    Serial.println("==========================");
}

void MotorControl::setSpeed(int speed) {
    _currentSpeed = _constrainSpeed(speed);
    
    if (_loggingEnabled) {
        Serial.print("[MotorControl] Speed set to: ");
        Serial.println(_currentSpeed);
    }
}

int MotorControl::getCurrentSpeed() const {
    return _currentSpeed;
}

int MotorControl::getLeftSpeed() const {
    return _leftSpeed;
}

int MotorControl::getRightSpeed() const {
    return _rightSpeed;
}

MotorState MotorControl::getCurrentState() const {
    return _currentState;
}

bool MotorControl::isMoving() const {
    return _currentState != MOTOR_STOP && _currentState != MOTOR_CALIBRATING;
}

bool MotorControl::isCalibrating() const {
    return _currentState == MOTOR_CALIBRATING;
}

// Private helper functions
void MotorControl::_setMotorPins(int leftSpeed, int rightSpeed) {
    _leftSpeed = leftSpeed;
    _rightSpeed = rightSpeed;
    
    _setMotorDirection(_left_in1_pin, _left_in2_pin, _left_pwm_pin, leftSpeed);
    _setMotorDirection(_right_in1_pin, _right_in2_pin, _right_pwm_pin, rightSpeed);
}

// Pololu TB9051FTG 3핀 제어 방식
void MotorControl::_setMotorDirection(int in1Pin, int in2Pin, int pwmPin, int speed) {
    if (speed > 0) {
        // 정방향
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        // 역방향
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, abs(speed));
    } else {
        // 정지
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, 0);
    }
}

void MotorControl::_updateState(MotorState newState) {
    _currentState = newState;
}

int MotorControl::_constrainSpeed(int speed) {
    return constrainSpeed(speed, _minSpeed, _maxSpeed);
}

bool MotorControl::_validateSpeed(int speed) {
    if (speed < _minSpeed || speed > _maxSpeed) {
        if (_loggingEnabled) {
            Serial.print("[MotorControl] Invalid speed: ");
            Serial.print(speed);
            Serial.print(" (valid range: ");
            Serial.print(_minSpeed);
            Serial.print("-");
            Serial.print(_maxSpeed);
            Serial.println(")");
        }
        return false;
    }
    return true;
}

void MotorControl::_updateSoftStop() {
    if (!_softStopInProgress) return;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - _softStopStartTime;
    const unsigned long softStopDuration = 500; // 500ms
    
    if (elapsed >= softStopDuration) {
        // 정지 완료
        stop();
    } else {
        // 점진적 감속
        float progress = (float)elapsed / softStopDuration;
        int newLeft = _softStopInitialLeft * (1.0 - progress);
        int newRight = _softStopInitialRight * (1.0 - progress);
        
        _setMotorPins(newLeft, newRight);
    }
}

void MotorControl::_logMotorAction(const char* action, int leftSpeed, int rightSpeed) {
    if (!_loggingEnabled) return;
    
    Serial.print("[MotorControl] ");
    Serial.print(action);
    Serial.print(" - Left: ");
    Serial.print(leftSpeed >= 0 ? "+" : "");
    Serial.print(leftSpeed);
    Serial.print(", Right: ");
    Serial.print(rightSpeed >= 0 ? "+" : "");
    Serial.println(rightSpeed);
}