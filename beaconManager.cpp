#include "beaconManager.h"
#include "utils.h"
#include <math.h>

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

    // 이전 값 초기화 (오래된 측정 제거)
    for (int i = 0; i < NUM_BEACONS; i++) {
        beacons[i].rssi = -100;
        beacons[i].distance = -1.0;
    }

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


                }
            }
        }
    }

    BLE.stopScan();
}

RobotPosition BeaconManager::calculatePosition() {
    // 유효한 비콘 인덱스 수집
    int indices[NUM_BEACONS];
    int count = 0;
    for (int i = 0; i < NUM_BEACONS; i++) {
        if (beacons[i].rssi > -100 && beacons[i].distance > 0) {
            indices[count++] = i;
        }
    }

    if (count < 3) {
        Serial.println("[BeaconManager] Not enough beacons for positioning");
        currentPosition.confidence = 0.0;
        return currentPosition;
    }

    // 1차: 전체 유효 비콘으로 최소자승 추정
    RobotPosition pos = leastSquaresPosition(indices, count);

    // 이상치 제거: 잔차가 가장 큰 비콘을 하나 제외하고 재추정 (필요 시)
    double maxResidual = -1.0;
    int worstIdxInIndices = -1;
    for (int k = 0; k < count; k++) {
        int i = indices[k];
        double d = calculateDistance(pos.x, pos.y, beacons[i].x, beacons[i].y);
        double r = beacons[i].distance;
        double res = fabs(d - r);
        if (res > maxResidual) {
            maxResidual = res;
            worstIdxInIndices = k;
        }
    }

    // 임계: 1.5m 이상 큰 잔차면 제거 후 재추정 (3개 이상 남아야 함)
    if (maxResidual > 1.5 && count >= 4 && worstIdxInIndices >= 0) {
        // 압축 이동
        for (int k = worstIdxInIndices; k < count - 1; k++) {
            indices[k] = indices[k + 1];
        }
        count -= 1;
        pos = leastSquaresPosition(indices, count);
    }

    currentPosition = pos;

    // 위치 범위 검증 (작업 공간 예시: 0~20m)
    if (currentPosition.x < -5 || currentPosition.x > 25 || 
        currentPosition.y < -5 || currentPosition.y > 25) {
        Serial.println("[BeaconManager] Position out of reasonable range");
        currentPosition.confidence *= 0.5; // 신뢰도 감소
    }

    return currentPosition;
}

RobotPosition BeaconManager::getCurrentPosition() {
    return currentPosition;
}

void BeaconManager::setBeaconPosition(int beaconIndex, double x, double y) {
    if (beaconIndex >= 0 && beaconIndex < NUM_BEACONS) {
        beacons[beaconIndex].x = x;
        beacons[beaconIndex].y = y;

    }
}

void BeaconManager::getGridCell(double cellSize, int& gridX, int& gridY) {
    gridX = worldToGrid(currentPosition.x, cellSize);
    gridY = worldToGrid(currentPosition.y, cellSize);
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
    
    if (fabs(det) < 0.0001) {
        // 행렬이 특이한 경우 (비콘이 일직선상에 있거나 너무 가까움)
        // 가중 평균으로 대체
        double w1 = 1.0 / (r1 * r1 + 1e-3);
        double w2 = 1.0 / (r2 * r2 + 1e-3);
        double w3 = 1.0 / (r3 * r3 + 1e-3);
        double totalW = w1 + w2 + w3;
        
        pos.x = (x1 * w1 + x2 * w2 + x3 * w3) / totalW;
        pos.y = (y1 * w1 + y2 * w2 + y3 * w3) / totalW;
        pos.confidence = 0.5; // 낮은 신뢰도
    } else {
        // 정확한 삼각측량
        pos.x = (E * D - B * F) / det;
        pos.y = (A * F - E * C) / det;
        
        // 신뢰도 계산 (거리 오차 기반) - utils 함수 사용
        double error1 = fabs(calculateDistance(pos.x, pos.y, x1, y1) - r1);
        double error2 = fabs(calculateDistance(pos.x, pos.y, x2, y2) - r2);
        double error3 = fabs(calculateDistance(pos.x, pos.y, x3, y3) - r3);
        
        double avgError = (error1 + error2 + error3) / 3.0;
        pos.confidence = max(0.0, 1.0 - avgError / 2.0); // 오차가 클수록 신뢰도 감소
    }
    
    return pos;
}

// 다중 비콘 최소자승 위치 추정
RobotPosition BeaconManager::leastSquaresPosition(const int indices[], int count) {
    RobotPosition pos;
    if (count < 3) {
        pos.x = 0.0;
        pos.y = 0.0;
        pos.confidence = 0.0;
        return pos;
    }

    const int k = 0; // 기준 인덱스는 배열 내 첫 항목
    const double xk = beacons[indices[k]].x;
    const double yk = beacons[indices[k]].y;
    const double rk = beacons[indices[k]].distance;

    double S_AA = 0.0, S_AB = 0.0, S_BB = 0.0, S_AC = 0.0, S_BC = 0.0;
    int used = 0;
    for (int t = 1; t < count; t++) {
        const int i = indices[t];
        const double xi = beacons[i].x;
        const double yi = beacons[i].y;
        const double ri = beacons[i].distance;
        if (ri <= 0) continue;
        const double A = 2.0 * (xi - xk);
        const double B = 2.0 * (yi - yk);
        const double C = (rk * rk - ri * ri) - (xk * xk - xi * xi) - (yk * yk - yi * yi);
        const double w = 1.0 / (ri * ri + 1e-3);
        S_AA += w * A * A;
        S_AB += w * A * B;
        S_BB += w * B * B;
        S_AC += w * A * C;
        S_BC += w * B * C;
        used++;
    }

    if (used < 2) {
        pos.x = 0.0;
        pos.y = 0.0;
        pos.confidence = 0.0;
        return pos;
    }

    const double det = S_AA * S_BB - S_AB * S_AB;
    if (fabs(det) < 1e-9) {
        // fallback: 가중 중심
        double wsum = 0.0, xsum = 0.0, ysum = 0.0;
        for (int t = 0; t < count; t++) {
            const int i = indices[t];
            const double xi = beacons[i].x;
            const double yi = beacons[i].y;
            const double ri = beacons[i].distance;
            if (ri <= 0) continue;
            const double w = 1.0 / (ri * ri + 1e-3);
            xsum += xi * w; ysum += yi * w; wsum += w;
        }
        pos.x = xsum / (wsum > 0 ? wsum : 1.0);
        pos.y = ysum / (wsum > 0 ? wsum : 1.0);
        pos.confidence = 0.5;
        return pos;
    }

    pos.x = ( S_BB * S_AC - S_AB * S_BC) / det;
    pos.y = ( S_AA * S_BC - S_AB * S_AC) / det;

    // 잔차 기반 신뢰도
    double errSum = 0.0;
    int v = 0;
    for (int t = 0; t < count; t++) {
        const int i = indices[t];
        const double xi = beacons[i].x;
        const double yi = beacons[i].y;
        const double ri = beacons[i].distance;
        if (ri <= 0) continue;
        const double d = calculateDistance(pos.x, pos.y, xi, yi);
        errSum += fabs(d - ri);
        v++;
    }
    const double meanErr = (v > 0) ? (errSum / v) : 1e9;
    if (meanErr > 10.0) {
        pos.confidence = 0.0;
    } else {
        pos.confidence = max(0.0, 1.0 - meanErr / 2.0);
    }

    return pos;
}
