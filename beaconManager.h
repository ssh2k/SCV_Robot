#ifndef BEACON_MANAGER_H
#define BEACON_MANAGER_H

#include <Arduino.h>
#include <ArduinoBLE.h>

// 비콘 정보 구조체
struct BeaconInfo {
    String address;
    int rssi;
    double distance;
    double x, y;  // 비콘의 고정 위치
};

// 로봇 위치 구조체
struct RobotPosition {
    double x, y;
    double confidence;  // 위치 신뢰도 (0.0 ~ 1.0)
};

class BeaconManager {
public:
    BeaconManager();
    ~BeaconManager();

    // 초기화
    bool begin();

    // 비콘 스캔 및 위치 계산
    void scanBeacons();

    RobotPosition calculatePosition();

    // 현재 위치 반환
    RobotPosition getCurrentPosition();

    // 비콘 위치 설정
    void setBeaconPosition(int beaconIndex, double x, double y);

    // RSSI -> 거리 변환
    double rssiToDistance(int rssi);

    // 삼각측량
    RobotPosition trilateration(BeaconInfo beacon1, BeaconInfo beacon2, BeaconInfo beacon3);

private:
    static const int NUM_BEACONS = 3;
    static const int SCAN_TIME_MS = 5000;  // 스캔 시간 밀리초

    BeaconInfo beacons[NUM_BEACONS];
    RobotPosition currentPosition;

    // 비콘 주소 (실제 주소로 변경)
    String beaconAddresses[NUM_BEACONS] = {
        "BE:AC:ON:01:02:03",
        "BE:AC:ON:04:05:06",
        "BE:AC:ON:07:08:09"
    };
};

#endif
