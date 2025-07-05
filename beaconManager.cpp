#include "beaconManager.h"

// 전역 변수로 콜백 함수에서 접근할 수 있도록
BeaconManager* globalBeaconManager = nullptr;

BeaconManager::BeaconManager() {
    pBLEScan = nullptr;
    callbacks = nullptr;
    globalBeaconManager = this;
    
    // 비콘 초기화
    for (int i = 0; i < NUM_BEACONS; i++) {
        beacons[i].address = beaconAddresses[i];
        beacons[i].rssi = -100;
        beacons[i].distance = 0.0;
        beacons[i].x = 0.0;
        beacons[i].y = 0.0;
    }
    
    // 현재 위치 초기화
    currentPosition.x = 0.0;
    currentPosition.y = 0.0;
    currentPosition.confidence = 0.0;
}

BeaconManager::~BeaconManager() {
    if (pBLEScan) {
        delete pBLEScan;
    }
    if (callbacks) {
        delete callbacks;
    }
}

void BeaconManager::begin() {
    Serial.println("[BeaconManager] Initializing BLE...");
    
    // BLE 초기화
    BLEDevice::init("SCV_Robot");
    
    // BLE 스캐너 생성
    pBLEScan = BLEDevice::getScan();
    callbacks = new MyAdvertisedDeviceCallbacks();
    pBLEScan->setAdvertisedDeviceCallbacks(callbacks);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    Serial.println("[BeaconManager] BLE initialized successfully");
}

void BeaconManager::scanBeacons() {
    Serial.println("[BeaconManager] Scanning for beacons...");
    
    // BLE 스캔 시작
    BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME, false);
    
    Serial.print("[BeaconManager] Scan complete. Found ");
    Serial.print(foundDevices.getCount());
    Serial.println(" devices");
    
    // 스캔 결과 처리
    for (int i = 0; i < foundDevices.getCount(); i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);
        String address = device.getAddress().toString().c_str();
        
        // 우리가 찾는 비콘인지 확인
        for (int j = 0; j < NUM_BEACONS; j++) {
            if (address.equals(beacons[j].address)) {
                beacons[j].rssi = device.getRSSI();
                beacons[j].distance = rssiToDistance(device.getRSSI());
                
                Serial.print("[BeaconManager] Found beacon ");
                Serial.print(j);
                Serial.print(": RSSI=");
                Serial.print(beacons[j].rssi);
                Serial.print(", Distance=");
                Serial.println(beacons[j].distance);
                break;
            }
        }
    }
}

RobotPosition BeaconManager::calculatePosition() {
    // 최소 3개의 비콘이 필요
    int validBeacons = 0;
    for (int i = 0; i < NUM_BEACONS; i++) {
        if (beacons[i].rssi > -100) {
            validBeacons++;
        }
    }
    
    if (validBeacons < 3) {
        Serial.println("[BeaconManager] Not enough beacons for positioning");
        currentPosition.confidence = 0.0;
        return currentPosition;
    }
    
    // 삼각측량 계산
    currentPosition = trilateration(beacons[0], beacons[1], beacons[2]);
    currentPosition.confidence = 0.8; // 기본 신뢰도
    
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
    // RSSI를 거리로 변환하는 공식
    // 실제 환경에 맞게 조정 필요
    double distance = pow(10, (-69 - rssi) / 20.0);
    return distance;
}

RobotPosition BeaconManager::trilateration(BeaconInfo beacon1, BeaconInfo beacon2, BeaconInfo beacon3) {
    // 삼각측량 알고리즘 구현
    // 간단한 가중 평균 방식 사용
    
    RobotPosition position;
    
    // 가중치 계산 (거리가 가까울수록 높은 가중치)
    double weight1 = 1.0 / (beacon1.distance + 0.1);
    double weight2 = 1.0 / (beacon2.distance + 0.1);
    double weight3 = 1.0 / (beacon3.distance + 0.1);
    
    double totalWeight = weight1 + weight2 + weight3;
    
    // 가중 평균으로 위치 계산
    position.x = (beacon1.x * weight1 + beacon2.x * weight2 + beacon3.x * weight3) / totalWeight;
    position.y = (beacon1.y * weight1 + beacon2.y * weight2 + beacon3.y * weight3) / totalWeight;
    
    return position;
}

// BLE 콜백 함수 구현
void BeaconManager::MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
    // 콜백에서 필요한 경우 추가 처리
    // 현재는 scanBeacons() 함수에서 처리
}
