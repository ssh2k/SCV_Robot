#include "beaconManager.h"

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
    Serial.println("[BeaconManager] Scanning for beacons...");

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

                    Serial.print("[BeaconManager] Found beacon ");
                    Serial.print(i);
                    Serial.print(": RSSI=");
                    Serial.print(beacons[i].rssi);
                    Serial.print(", Distance=");
                    Serial.println(beacons[i].distance);
                }
            }
        }
    }

    BLE.stopScan();
}

RobotPosition BeaconManager::calculatePosition() {
    int validBeacons = 0;
    for (int i = 0; i < NUM_BEACONS; i++) {
        if (beacons[i].rssi > -100) validBeacons++;
    }

    if (validBeacons < 3) {
        Serial.println("[BeaconManager] Not enough beacons for positioning");
        currentPosition.confidence = 0.0;
        return currentPosition;
    }

    currentPosition = trilateration(beacons[0], beacons[1], beacons[2]);
    currentPosition.confidence = 0.8;

    Serial.print("[BeaconManager] Position calculated: (");
    Serial.print(currentPosition.x);
    Serial.print(", ");
    Serial.print(currentPosition.y);
    Serial.print("), Confidence: ");
    Serial.println(currentPosition.confidence);

    return currentPosition;
}

RobotPosition BeaconManager::getCurrentPosition() {
    return currentPosition;
}

void BeaconManager::setBeaconPosition(int beaconIndex, double x, double y) {
    if (beaconIndex >= 0 && beaconIndex < NUM_BEACONS) {
        beacons[beaconIndex].x = x;
        beacons[beaconIndex].y = y;
        Serial.print("[BeaconManager] Beacon ");
        Serial.print(beaconIndex);
        Serial.print(" position set to (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.println(")");
    }
}

double BeaconManager::rssiToDistance(int rssi) {
    double txPower = -69; // 기준 RSSI 값 (1m 거리)
    double n = 2.0; // 경로 손실 지수 (환경에 따라 조정)

    double distance = pow(10.0, (txPower - rssi) / (10 * n));
    return distance;
}

RobotPosition BeaconManager::trilateration(BeaconInfo b1, BeaconInfo b2, BeaconInfo b3) {
    RobotPosition pos;

    double w1 = 1.0 / (b1.distance + 0.1);
    double w2 = 1.0 / (b2.distance + 0.1);
    double w3 = 1.0 / (b3.distance + 0.1);

    double totalW = w1 + w2 + w3;

    pos.x = (b1.x * w1 + b2.x * w2 + b3.x * w3) / totalW;
    pos.y = (b1.y * w1 + b2.y * w2 + b3.y * w3) / totalW;

    return pos;
}
