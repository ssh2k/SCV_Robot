#include "motorControl.h"
#include "beaconManager.h"
#include "pathfinder.h"
#include "communication.h"
#include "mapLearner.h"
#include "utils.h"

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

// --- 시스템 설정 ---
const unsigned long POSITION_UPDATE_INTERVAL = 1000; // 1초
const unsigned long BEACON_SCAN_INTERVAL = 5000;     // 5초
const double WAYPOINT_REACH_THRESHOLD = 0.3;        // 30cm
const double ROTATION_THRESHOLD = 0.2;              // 약 11도
const double LARGE_ROTATION_THRESHOLD = 0.8;        // 약 45도

// --- 객체 생성 ---
MotorControl motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);
BeaconManager beaconManager;
Pathfinder pathfinder(GRID_WIDTH, GRID_HEIGHT);
Communication communication;
MapLearner mapLearner(&pathfinder, GRID_WIDTH, GRID_HEIGHT);

// --- 전역 변수 ---
RobotPosition currentPosition;
std::vector<PathPoint> currentPath;
int currentPathIndex = 0;
bool isNavigating = false;
bool emergencyStop = false;
bool isMapLearning = false;

// --- 상태 변수 ---
unsigned long lastPositionUpdate = 0;
unsigned long lastBeaconScan = 0;
unsigned long lastDebugTime = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial); // 시리얼 포트가 열릴 때까지 대기

    Serial.println("=== SCV Robot Initialization ===");
    
    // 1. 모터 초기화
    Serial.println("[Main] Initializing motor control...");
    motor.begin();
    motor.setMaxSpeed(255);
    motor.setMinSpeed(50);
    
    // 2. 비콘 관리자 초기화
    Serial.println("[Main] Initializing beacon manager...");
    beaconManager.begin();
    
    // 비콘 위치 설정 (실제 환경에 맞게 수정)
    beaconManager.setBeaconPosition(0, 0.0, 0.0);      // 비콘 1: (0, 0)
    beaconManager.setBeaconPosition(1, 10.0, 0.0);     // 비콘 2: (10, 0)
    beaconManager.setBeaconPosition(2, 5.0, 10.0);     // 비콘 3: (5, 10)
    
    // 3. 경로 탐색 초기화
    Serial.println("[Main] Initializing pathfinder...");
    pathfinder.begin();
    
    // 4. 맵 학습 초기화
    Serial.println("[Main] Initializing map learner...");
    mapLearner.begin();
    
    // 5. 기본 장애물 설정 (외벽)
    setupBasicObstacles();
    
    // 6. 통신 초기화
    Serial.println("[Main] Initializing communication...");
    communication.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // 7. 콜백 함수 설정
    setupCallbacks();
    
    Serial.println("[Main] Initialization complete!");
    Serial.println("=== SCV Robot Ready ===");
}

void loop() {
    unsigned long currentTime = millis();
    
    // 1. WiFi 연결 확인
    if (!communication.isConnected()) {
        Serial.println("[Main] WiFi disconnected. Attempting to reconnect...");
        // 재연결 로직은 communication 클래스에서 처리
    }
    
    // 2. 웹서버 클라이언트 처리
    communication.handleClient();
    
    // 3. 비콘 스캔 및 위치 업데이트
    if (currentTime - lastBeaconScan >= BEACON_SCAN_INTERVAL) {
        updatePositionFromBeacons();
        lastBeaconScan = currentTime;
    }
    
    // 4. 위치 기반 상태 업데이트
    if (currentTime - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        updateRobotStatus();
        lastPositionUpdate = currentTime;
    }
    
    // 5. 맵 학습 처리
    if (isMapLearning) {
        handleMapLearning();
    }
    
    // 6. 경로 탐색 및 이동
    if (isNavigating && !emergencyStop) {
        navigateToTarget();
    }
    
    // 7. 긴급 정지 처리
    if (emergencyStop) {
        motor.emergencyStop();
        isNavigating = false;
        isMapLearning = false;
        emergencyStop = false; // 한 번만 처리
    }
    
    // 8. 디버그 정보 출력
    if (currentTime - lastDebugTime >= 5000) { // 5초마다
        printDebugInfo();
        lastDebugTime = currentTime;
    }
    
    delay(100); // 100ms 딜레이
}

// --- 초기화 함수들 ---

void setupBasicObstacles() {
    // 외벽 설정
    for (int x = 0; x < GRID_WIDTH; x++) {
        pathfinder.setObstacle(x, 0, true);           // 하단 벽
        pathfinder.setObstacle(x, GRID_HEIGHT-1, true); // 상단 벽
    }
    for (int y = 0; y < GRID_HEIGHT; y++) {
        pathfinder.setObstacle(0, y, true);           // 좌측 벽
        pathfinder.setObstacle(GRID_WIDTH-1, y, true); // 우측 벽
    }
}

void setupCallbacks() {
    // Communication 모듈에 콜백 함수 설정
    communication.setCommandCallback(handleCommand);
    communication.setStatusCallback(getRobotStatus);
}

// --- 명령 처리 함수들 ---

void handleCommand(const MoveCommand& command) {
    switch (command.type) {
        case CMD_MOVE_TO_POSITION:
            moveToPosition(command.x, command.y, command.speed);
            break;
            
        case CMD_EMERGENCY_STOP:
            emergencyStop = true;
            motor.emergencyStop();
            isNavigating = false;
            isMapLearning = false;
            break;
            
        case CMD_SET_SPEED:
            motor.setSpeed(command.speed);
            break;
            
        case CMD_LEARN_MAP:
            isMapLearning = true;
            isNavigating = false;
            motor.softStop();
            break;
            
        case CMD_APPLY_LEARNED_MAP:
            mapLearner.applyLearnedMap();
            break;
            
        case CMD_CLEAR_MAP:
            mapLearner.begin();
            setupBasicObstacles();
            break;
            
        default:
            break;
    }
}

RobotStatus getRobotStatus() {
    RobotStatus status;
    status.currentX = currentPosition.x;
    status.currentY = currentPosition.y;
    status.isMoving = isNavigating;
    status.isEmergencyStop = emergencyStop;
    status.isMapLearning = isMapLearning;
    status.currentSpeed = motor.getCurrentSpeed();
    status.batteryLevel = getBatteryLevel();
    status.lastError = "";
    
    // 목표 위치 설정
    if (!currentPath.empty() && currentPathIndex < currentPath.size()) {
        PathPoint target = currentPath[currentPathIndex];
        status.targetX = gridToWorld(target.x, GRID_CELL_SIZE);
        status.targetY = gridToWorld(target.y, GRID_CELL_SIZE);
    } else {
        status.targetX = currentPosition.x;
        status.targetY = currentPosition.y;
    }
    
    return status;
}

// --- 위치 업데이트 함수들 ---

void updatePositionFromBeacons() {
    beaconManager.scanBeacons();
    currentPosition = beaconManager.calculatePosition();
    
    // 위치 신뢰도가 낮으면 경고
    if (currentPosition.confidence < 0.5) {
        Serial.println("[Main] Warning: Low position confidence");
    }
}

void updateRobotStatus() {
    RobotStatus status = getRobotStatus();
    communication.updateStatus(status);
}

// --- 맵 학습 관련 함수들 ---

void handleMapLearning() {
    // 맵 학습 실행
    mapLearner.learnMap();
    
    // 학습된 맵을 pathfinder에 적용
    mapLearner.applyLearnedMap();
    
    // 맵 학습 완료
    isMapLearning = false;
}

// --- 네비게이션 관련 함수들 ---

void navigateToTarget() {
    if (currentPath.empty() || currentPathIndex >= currentPath.size()) {
        // 경로 완료
        motor.softStop();
        isNavigating = false;
        return;
    }
    
    // 현재 목표점
    PathPoint target = currentPath[currentPathIndex];
    double targetX = gridToWorld(target.x, GRID_CELL_SIZE);
    double targetY = gridToWorld(target.y, GRID_CELL_SIZE);
    
    // 현재 위치에서 목표점까지의 거리 계산 (utils 함수 사용)
    double distance = calculateDistance(currentPosition.x, currentPosition.y, targetX, targetY);
    
    if (distance < WAYPOINT_REACH_THRESHOLD) {
        // 다음 경로점으로 이동
        currentPathIndex++;
        return;
    }
    
    // 목표점까지의 방향 계산 및 이동 제어
    executeMovementToTarget(targetX, targetY);
}

void executeMovementToTarget(double targetX, double targetY) {
    // 목표점까지의 방향 계산
    double angle = atan2(targetY - currentPosition.y, targetX - currentPosition.x);
    
    // 로봇의 현재 방향 (실제로는 자이로스코프나 엔코더 필요)
    double robotAngle = getCurrentRobotAngle();
    
    // 회전 각도 계산
    double rotationAngle = angle - robotAngle;
    
    // 각도 정규화 (-π ~ π)
    normalizeAngle(rotationAngle);
    
    // 회전 및 이동 제어
    int baseSpeed = 200;
    int rotationSpeed = 150;
    
    if (abs(rotationAngle) > ROTATION_THRESHOLD) {
        // 회전이 필요한 경우
        if (abs(rotationAngle) > LARGE_ROTATION_THRESHOLD) {
            // 제자리 회전
            executeRotation(rotationAngle, rotationSpeed);
        } else {
            // 곡선 이동 (회전하면서 전진)
            executeCurvedMovement(rotationAngle, baseSpeed);
        }
    } else {
        // 직진
        motor.forward(baseSpeed);
    }
}

void executeRotation(double rotationAngle, int rotationSpeed) {
    if (rotationAngle > 0) {
        motor.turnLeft(rotationSpeed);
    } else {
        motor.turnRight(rotationSpeed);
    }
}

void executeCurvedMovement(double rotationAngle, int baseSpeed) {
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

void printDebugInfo() {
    Serial.println("=== SCV Robot Debug Info ===");
    Serial.print("Position: (");
    Serial.print(currentPosition.x, 2);
    Serial.print(", ");
    Serial.print(currentPosition.y, 2);
    Serial.print("), Confidence: ");
    Serial.println(currentPosition.confidence, 2);
    Serial.print("Navigation: ");
    Serial.println(isNavigating ? "Active" : "Idle");
    Serial.print("Motor State: ");
    Serial.println(motor.getCurrentState());
    Serial.println("============================");
}

// --- 외부 명령 처리 함수들 ---

void moveToPosition(double x, double y, int speed) {
    
    // 그리드 좌표로 변환 (utils 함수 사용)
    int gridX = worldToGrid(x, GRID_CELL_SIZE);
    int gridY = worldToGrid(y, GRID_CELL_SIZE);
    int currentGridX = worldToGrid(currentPosition.x, GRID_CELL_SIZE);
    int currentGridY = worldToGrid(currentPosition.y, GRID_CELL_SIZE);
    
    // 경로 찾기
    currentPath = pathfinder.findPath(currentGridX, currentGridY, gridX, gridY);
    
    if (currentPath.empty()) {
        communication.setError("No path found to target");
        return;
    }
    
    // 경로 최적화
    currentPath = pathfinder.optimizePath(currentPath);
    
    // 네비게이션 시작
    currentPathIndex = 0;
    isNavigating = true;
    emergencyStop = false;
}
