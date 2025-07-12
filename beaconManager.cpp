#include "beaconManager.h"
#include "utils.h"

BeaconManager::BeaconManager() {
    // 비콘 초기값 설정
    for (int i = 0; i < NUM_BEACONS; i++) {
        beacons[i].address = beaconAddresses[i];
        beacons[i].rssi = -100;
        beacons[i].distance = 0.0;
        beacons[i].x = 0.0;
        beacons[i].y = 0.0;
    }

    currentPosition = {0.0, 0.0, 0.0};
}

BeaconManager::~BeaconManager() {
    // ArduinoBLE에는 별도 삭제 필요 없음
}

bool BeaconManager::begin() {
    Serial.println("[BeaconManager] Initializing BLE...");

    if (!BLE.begin()) {
        Serial.println("[BeaconManager] Failed to initialize BLE!");
        return false;
    }

    Serial.println("[BeaconManager] BLE initialized successfully");
    return true;
}

void BeaconManager::scanBeacons() {
    BLE.scanForUuid("FEAA"); // Eddystone UUID 예시, 필요 시 수정

    unsigned long startTime = millis();

    while (millis() - startTime < SCAN_TIME_MS) {
        BLEDevice peripheral = BLE.available();

        if (peripheral) {
            String address = peripheral.address();

            // 찾는 비콘인지 확인
            for (int i = 0; i < NUM_BEACONS; i++) {
                if (address.equalsIgnoreCase(beacons[i].address)) {
                    beacons[i].rssi = peripheral.rssi();
                    beacons[i].distance = rssiToDistance(beacons[i].rssi);

                    // 비콘 발견 로그 제거 - 필요시에만 출력
                }
            }
        }
    }

    BLE.stopScan();
}

RobotPosition BeaconManager::calculatePosition() {
    int validBeacons = 0;
    double totalConfidence = 0.0;
    
    // 유효한 비콘 개수 확인 및 거리 검증
    for (int i = 0; i < NUM_BEACONS; i++) {
        if (beacons[i].rssi > -100 && beacons[i].distance > 0) {
            validBeacons++;
            totalConfidence += 1.0;
        }
    }

    if (validBeacons < 3) {
        Serial.println("[BeaconManager] Not enough beacons for positioning");
        currentPosition.confidence = 0.0;
        return currentPosition;
    }

    // 삼각측량으로 위치 계산
    currentPosition = trilateration(beacons[0], beacons[1], beacons[2]);
    
    // 신뢰도 조정 (유효한 비콘 개수와 거리 오차 기반)
    double beaconConfidence = totalConfidence / 3.0;
    currentPosition.confidence = min(currentPosition.confidence, beaconConfidence);
    
    // 위치 범위 검증
    if (currentPosition.x < -5 || currentPosition.x > 15 || 
        currentPosition.y < -5 || currentPosition.y > 15) {
        Serial.println("[BeaconManager] Position out of reasonable range");
        currentPosition.confidence *= 0.5; // 신뢰도 감소
    }

    // 위치 계산 결과 로그 제거

    return currentPosition;
}

RobotPosition BeaconManager::getCurrentPosition() {
    return currentPosition;
}

void BeaconManager::setBeaconPosition(int beaconIndex, double x, double y) {
    if (beaconIndex >= 0 && beaconIndex < NUM_BEACONS) {
        beacons[beaconIndex].x = x;
        beacons[beaconIndex].y = y;
        // 비콘 위치 설정 로그 제거
    }
}

double BeaconManager::rssiToDistance(int rssi) {
    // 개선된 RSSI → 거리 변환
    double txPower = -69; // 기준 RSSI 값 (1m 거리)
    double n = 2.0; // 경로 손실 지수 (환경에 따라 조정)
    
    // RSSI 값 검증
    if (rssi > -30 || rssi < -100) {
        return -1.0; // 유효하지 않은 RSSI
    }
    
    double distance = pow(10.0, (txPower - rssi) / (10 * n));
    
    // 거리 제한 (0.1m ~ 20m)
    if (distance < 0.1) distance = 0.1;
    if (distance > 20.0) distance = 20.0;
    
    return distance;
}

RobotPosition BeaconManager::trilateration(BeaconInfo b1, BeaconInfo b2, BeaconInfo b3) {
    RobotPosition pos;
    
    // 거리 정보 검증
    if (b1.distance <= 0 || b2.distance <= 0 || b3.distance <= 0) {
        pos.x = 0.0;
        pos.y = 0.0;
        pos.confidence = 0.0;
        return pos;
    }
    
    // 실제 삼각측량 공식 구현
    // (x-x1)² + (y-y1)² = r1²
    // (x-x2)² + (y-y2)² = r2²
    // (x-x3)² + (y-y3)² = r3²
    
    double x1 = b1.x, y1 = b1.y, r1 = b1.distance;
    double x2 = b2.x, y2 = b2.y, r2 = b2.distance;
    double x3 = b3.x, y3 = b3.y, r3 = b3.distance;
    
    // 선형 방정식으로 변환
    double A = 2 * (x2 - x1);
    double B = 2 * (y2 - y1);
    double C = 2 * (x3 - x1);
    double D = 2 * (y3 - y1);
    
    double E = r1 * r1 - r2 * r2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;
    double F = r1 * r1 - r3 * r3 - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;
    
    // 행렬식 계산
    double det = A * D - B * C;
    
    if (abs(det) < 0.0001) {
        // 행렬이 특이한 경우 (비콘이 일직선상에 있거나 너무 가까움)
        // 가중 평균으로 대체
        double w1 = 1.0 / (r1 + 0.1);
        double w2 = 1.0 / (r2 + 0.1);
        double w3 = 1.0 / (r3 + 0.1);
        double totalW = w1 + w2 + w3;
        
        pos.x = (x1 * w1 + x2 * w2 + x3 * w3) / totalW;
        pos.y = (y1 * w1 + y2 * w2 + y3 * w3) / totalW;
        pos.confidence = 0.5; // 낮은 신뢰도
    } else {
        // 정확한 삼각측량
        pos.x = (E * D - B * F) / det;
        pos.y = (A * F - E * C) / det;
        
        // 신뢰도 계산 (거리 오차 기반) - utils 함수 사용
        double error1 = abs(calculateDistance(pos.x, pos.y, x1, y1) - r1);
        double error2 = abs(calculateDistance(pos.x, pos.y, x2, y2) - r2);
        double error3 = abs(calculateDistance(pos.x, pos.y, x3, y3) - r3);
        
        double avgError = (error1 + error2 + error3) / 3.0;
        pos.confidence = max(0.0, 1.0 - avgError / 2.0); // 오차가 클수록 신뢰도 감소
    }
    
    return pos;
}
