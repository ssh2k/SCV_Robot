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