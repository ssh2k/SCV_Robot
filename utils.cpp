#include "utils.h"

// 거리 계산 함수
double calculateDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 각도 정규화 함수 (-π ~ π 범위로)
void normalizeAngle(double& angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
}

// 현재 로봇 각도 반환 (실제 센서 연동 필요)
double getCurrentRobotAngle() {
    // TODO: 자이로스코프나 엔코더 센서 연동 필요
    // 현재는 0도로 고정 (정면 방향)
    return 0.0;
}

// 배터리 레벨 반환 (실제 센서 연동 필요)
double getBatteryLevel() {
    // TODO: 배터리 모니터링 센서 연동 필요
    // 현재는 100%로 고정
    return 100.0;
}

// 속도 제한 함수
int constrainSpeed(int speed, int minSpeed, int maxSpeed) {
    if (speed < minSpeed) return minSpeed;
    if (speed > maxSpeed) return maxSpeed;
    return speed;
}

// 좌표 유효성 검사
bool isValidCoordinate(double x, double y, double maxX, double maxY) {
    return (x >= 0 && x <= maxX && y >= 0 && y <= maxY);
}

// 그리드 좌표 변환
int worldToGrid(double worldCoord, double cellSize) {
    return (int)(worldCoord / cellSize);
}

double gridToWorld(int gridCoord, double cellSize) {
    return gridCoord * cellSize;
}

// Pololu TB9051FTG 모터 드라이버 관련 유틸리티 함수들

// 모터 방향 설정 헬퍼 함수
void setMotorDirection(int in1Pin, int in2Pin, int pwmPin, int speed) {
#ifdef ARDUINO
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
#else
    (void)in1Pin; (void)in2Pin; (void)pwmPin; (void)speed;
#endif
}

// 모터 안전 정지 함수
void safeMotorStop(int in1Pin, int in2Pin, int pwmPin) {
#ifdef ARDUINO
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
#else
    (void)in1Pin; (void)in2Pin; (void)pwmPin;
#endif
}

// 모터 테스트 함수
void testMotor(int in1Pin, int in2Pin, int pwmPin, const char* motorName) {
#ifdef ARDUINO
    Serial.print("[Utils] Testing ");
    Serial.println(motorName);
    
    // 정방향 테스트 (50% 속도)
    Serial.println("  - Forward test (50% speed)");
    setMotorDirection(in1Pin, in2Pin, pwmPin, 128);
    delay(1000);
    
    // 정지
    Serial.println("  - Stop");
    safeMotorStop(in1Pin, in2Pin, pwmPin);
    delay(500);
    
    // 역방향 테스트 (50% 속도)
    Serial.println("  - Backward test (50% speed)");
    setMotorDirection(in1Pin, in2Pin, pwmPin, -128);
    delay(1000);
    
    // 정지
    Serial.println("  - Stop");
    safeMotorStop(in1Pin, in2Pin, pwmPin);
    delay(500);
    
    Serial.println("  - Test completed");
#else
    (void)in1Pin; (void)in2Pin; (void)pwmPin; (void)motorName;
#endif
}

// 모터 속도 매핑 함수 (0-100%를 0-255로 변환)
int mapSpeedPercent(int percent) {
    if (percent < 0) return 0;
    if (percent > 100) return 255;
#ifdef ARDUINO
    return map(percent, 0, 100, 0, 255);
#else
    // 선형 매핑 대체
    return static_cast<int>(percent * 2.55);
#endif
}

// 모터 방향 검증 함수
bool isValidMotorDirection(int speed) {
    return (speed >= -255 && speed <= 255);
} 