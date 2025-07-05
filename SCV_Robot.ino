#include "motorControl.h"
#include "beaconManager.h"
#include "pathfinder.h"
#include "communication.h"

// --- 핀 설정 ---
// 아두이노 R4 보드의 실제 연결된 핀 번호로 수정하세요.
// PWM 기능이 있는 핀(~)을 PWM 핀으로 사용해야 합니다. (예: 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PWM_PIN  9
#define LEFT_MOTOR_DIR_PIN  8
#define RIGHT_MOTOR_PWM_PIN 10
#define RIGHT_MOTOR_DIR_PIN 11

// --- WiFi 설정 ---
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// --- 그리드 설정 ---
const int GRID_WIDTH = 20;
const int GRID_HEIGHT = 20;
const double GRID_CELL_SIZE = 0.5; // 미터 단위

// --- 객체 생성 ---
MotorControl motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);
BeaconManager beaconManager;
Pathfinder pathfinder(GRID_WIDTH, GRID_HEIGHT);
Communication communication;

// --- 전역 변수 ---
RobotPosition currentPosition;
std::vector<PathPoint> currentPath;
int currentPathIndex = 0;
bool isNavigating = false;
bool emergencyStop = false;

// --- 상태 변수 ---
unsigned long lastPositionUpdate = 0;
unsigned long lastBeaconScan = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 1000; // 1초
const unsigned long BEACON_SCAN_INTERVAL = 5000;     // 5초

void setup() {
    Serial.begin(9600);
    while (!Serial); // 시리얼 포트가 열릴 때까지 대기

    Serial.println("=== SCV Robot Initialization ===");
    
    // 모터 초기화
    Serial.println("[Main] Initializing motor control...");
    motor.begin();
    
    // 비콘 관리자 초기화
    Serial.println("[Main] Initializing beacon manager...");
    beaconManager.begin();
    
    // 비콘 위치 설정 (실제 환경에 맞게 수정)
    beaconManager.setBeaconPosition(0, 0.0, 0.0);      // 비콘 1: (0, 0)
    beaconManager.setBeaconPosition(1, 10.0, 0.0);     // 비콘 2: (10, 0)
    beaconManager.setBeaconPosition(2, 5.0, 10.0);     // 비콘 3: (5, 10)
    
    // 경로 탐색 초기화
    Serial.println("[Main] Initializing pathfinder...");
    pathfinder.begin();
    
    // 장애물 설정 (예시)
    setupObstacles();
    
    // 통신 초기화
    Serial.println("[Main] Initializing communication...");
    communication.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.println("[Main] Initialization complete!");
    Serial.println("=== SCV Robot Ready ===");
}

void loop() {
    unsigned long currentTime = millis();
    
    // WiFi 연결 확인
    if (!communication.isConnected()) {
        Serial.println("[Main] WiFi disconnected. Attempting to reconnect...");
        // 재연결 로직은 communication 클래스에서 처리
    }
    
    // 웹서버 클라이언트 처리
    communication.handleClient();
    
    // 비콘 스캔 및 위치 업데이트
    if (currentTime - lastBeaconScan >= BEACON_SCAN_INTERVAL) {
        Serial.println("[Main] Scanning beacons for position update...");
        beaconManager.scanBeacons();
        currentPosition = beaconManager.calculatePosition();
        lastBeaconScan = currentTime;
        
        // 위치 신뢰도가 낮으면 경고
        if (currentPosition.confidence < 0.5) {
            Serial.println("[Main] Warning: Low position confidence");
        }
    }
    
    // 위치 기반 상태 업데이트
    if (currentTime - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        updateRobotStatus();
        lastPositionUpdate = currentTime;
    }
    
    // 경로 탐색 및 이동
    if (isNavigating && !emergencyStop) {
        navigateToTarget();
    }
    
    // 긴급 정지 처리
    if (emergencyStop) {
        motor.emergencyStop();
        isNavigating = false;
        emergencyStop = false; // 한 번만 처리
        Serial.println("[Main] Emergency stop processed");
    }
    
    delay(100); // 100ms 딜레이
}

void setupObstacles() {
    // 예시 장애물 설정 (실제 환경에 맞게 수정)
    // 벽이나 장애물이 있는 위치를 true로 설정
    
    // 외벽 설정
    for (int x = 0; x < GRID_WIDTH; x++) {
        pathfinder.setObstacle(x, 0, true);           // 하단 벽
        pathfinder.setObstacle(x, GRID_HEIGHT-1, true); // 상단 벽
    }
    for (int y = 0; y < GRID_HEIGHT; y++) {
        pathfinder.setObstacle(0, y, true);           // 좌측 벽
        pathfinder.setObstacle(GRID_WIDTH-1, y, true); // 우측 벽
    }
    
    // 중앙 장애물 예시
    pathfinder.setObstacle(5, 5, true);
    pathfinder.setObstacle(5, 6, true);
    pathfinder.setObstacle(6, 5, true);
    
    Serial.println("[Main] Obstacles configured");
}

void updateRobotStatus() {
    RobotStatus status;
    status.currentX = currentPosition.x;
    status.currentY = currentPosition.y;
    status.isMoving = isNavigating;
    status.isEmergencyStop = emergencyStop;
    status.currentSpeed = 200; // 기본 속도
    status.batteryLevel = 100.0; // 배터리 레벨 (실제로는 배터리 모니터링 필요)
    status.lastError = "";
    
    // 목표 위치 설정
    if (!currentPath.empty() && currentPathIndex < currentPath.size()) {
        PathPoint target = currentPath[currentPathIndex];
        status.targetX = target.x * GRID_CELL_SIZE;
        status.targetY = target.y * GRID_CELL_SIZE;
    } else {
        status.targetX = currentPosition.x;
        status.targetY = currentPosition.y;
    }
    
    communication.updateStatus(status);
}

void navigateToTarget() {
    if (currentPath.empty() || currentPathIndex >= currentPath.size()) {
        // 경로 완료
        motor.softStop();
        isNavigating = false;
        Serial.println("[Main] Navigation completed");
        return;
    }
    
    // 현재 목표점
    PathPoint target = currentPath[currentPathIndex];
    double targetX = target.x * GRID_CELL_SIZE;
    double targetY = target.y * GRID_CELL_SIZE;
    
    // 현재 위치에서 목표점까지의 거리 계산
    double distance = sqrt(pow(targetX - currentPosition.x, 2) + pow(targetY - currentPosition.y, 2));
    
    if (distance < 0.3) { // 30cm 이내에 도달
        // 다음 경로점으로 이동
        currentPathIndex++;
        Serial.print("[Main] Reached waypoint ");
        Serial.print(currentPathIndex - 1);
        Serial.print("/");
        Serial.println(currentPath.size());
        return;
    }
    
    // 목표점까지의 방향 계산
    double angle = atan2(targetY - currentPosition.y, targetX - currentPosition.x);
    
    // 로봇의 현재 방향 (간단한 구현, 실제로는 자이로스코프나 엔코더 필요)
    double robotAngle = 0.0; // 실제로는 현재 방향을 추적해야 함
    
    // 회전 각도 계산
    double rotationAngle = angle - robotAngle;
    
    // 각도 정규화 (-π ~ π)
    while (rotationAngle > PI) rotationAngle -= 2 * PI;
    while (rotationAngle < -PI) rotationAngle += 2 * PI;
    
    // 회전 및 이동 제어 (개선된 로직)
    int baseSpeed = 200;
    int rotationSpeed = 150;
    double rotationThreshold = 0.2; // 약 11도
    
    if (abs(rotationAngle) > rotationThreshold) {
        // 회전이 필요한 경우
        if (abs(rotationAngle) > 0.8) { // 약 45도 이상
            // 제자리 회전
            if (rotationAngle > 0) {
                motor.turnLeft(rotationSpeed);
            } else {
                motor.turnRight(rotationSpeed);
            }
        } else {
            // 곡선 이동 (회전하면서 전진)
            int leftSpeed, rightSpeed;
            if (rotationAngle > 0) {
                // 좌회전하면서 전진
                leftSpeed = baseSpeed * 0.6;
                rightSpeed = baseSpeed;
                motor.curveLeft(leftSpeed, rightSpeed);
            } else {
                // 우회전하면서 전진
                leftSpeed = baseSpeed;
                rightSpeed = baseSpeed * 0.6;
                motor.curveRight(leftSpeed, rightSpeed);
            }
        }
    } else {
        // 직진
        motor.forward(baseSpeed);
    }
    
    // 디버그 정보 출력 (주기적으로)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) { // 1초마다
        Serial.print("[Main] Navigation - Distance: ");
        Serial.print(distance);
        Serial.print("m, Angle: ");
        Serial.print(rotationAngle * 180 / PI);
        Serial.print("°, State: ");
        Serial.println(motor.getCurrentState());
        lastDebugTime = millis();
    }
}

// 외부에서 호출할 수 있는 함수들
void moveToPosition(double x, double y, int speed) {
    Serial.print("[Main] Moving to position (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(") at speed ");
    Serial.println(speed);
    
    // 그리드 좌표로 변환
    int gridX = (int)(x / GRID_CELL_SIZE);
    int gridY = (int)(y / GRID_CELL_SIZE);
    int currentGridX = (int)(currentPosition.x / GRID_CELL_SIZE);
    int currentGridY = (int)(currentPosition.y / GRID_CELL_SIZE);
    
    // 경로 찾기
    currentPath = pathfinder.findPath(currentGridX, currentGridY, gridX, gridY);
    
    if (currentPath.empty()) {
        Serial.println("[Main] No path found to target");
        communication.setError("No path found to target");
        return;
    }
    
    // 경로 최적화
    currentPath = pathfinder.optimizePath(currentPath);
    
    // 경로 출력
    pathfinder.printPath(currentPath);
    
    // 네비게이션 시작
    currentPathIndex = 0;
    isNavigating = true;
    emergencyStop = false;
    
    Serial.println("[Main] Navigation started");
}

void emergencyStopRobot() {
    Serial.println("[Main] Emergency stop activated");
    emergencyStop = true;
    motor.emergencyStop();
    isNavigating = false;
}

void setRobotSpeed(int speed) {
    Serial.print("[Main] Setting robot speed to ");
    Serial.println(speed);
    // 속도 설정 로직 (필요시 모터 제어에 반영)
}
