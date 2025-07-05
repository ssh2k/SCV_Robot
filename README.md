# SCV Robot - 이동형 선반 로봇

아두이노 WiFi R4 보드를 기반으로 한 자율주행 이동형 선반 로봇입니다. BLE 비콘을 통한 위치 인식과 A* 알고리즘을 사용한 경로 탐색, WiFi를 통한 원격 제어 기능을 제공합니다.

## 주요 기능

### 🤖 로봇 제어
- **모터 제어**: PWM을 통한 정밀한 속도 제어
- **방향 제어**: 전진, 후진, 좌회전, 우회전, 곡선 이동
- **안전 기능**: 긴급 정지, 부드러운 정지, 속도 제한
- **캘리브레이션**: 모터 캘리브레이션 및 테스트 기능

### 📡 위치 인식
- **BLE 비콘**: 3개의 비콘을 통한 삼각측량
- **실시간 위치 추적**: RSSI 기반 거리 측정
- **위치 신뢰도**: 측정 정확도 평가

### 🗺️ 경로 탐색
- **A* 알고리즘**: 최적 경로 탐색
- **장애물 회피**: 동적 장애물 감지 및 회피
- **경로 최적화**: 불필요한 경유점 제거

### 🌐 원격 제어
- **WiFi 통신**: HTTP API 기반 명령 수신
- **REST API**: JSON 기반 명령 처리 및 응답
- **콜백 패턴**: 메인 컨트롤러와 느슨한 결합
- **입력 검증**: 명령 데이터 유효성 검사

## 시스템 아키텍처

```
Next.js App (UI/UX)
    ↓ HTTP API 호출
Communication (명령 수신/파싱)
    ↓ 콜백 함수
Main Controller (로봇 제어 로직)
    ↓ 모듈 호출
MotorControl | BeaconManager | Pathfinder
```

## 하드웨어 요구사항

### 필수 구성품
- **아두이노 WiFi R4** (메인 컨트롤러)
- **DC 모터 2개** (좌우 바퀴 구동)
- **모터 드라이버** (L298N 또는 H-브리지)
- **BLE 비콘 3개** (위치 인식용)
- **배터리** (12V 권장)

### 선택 구성품
- **초음파 센서** (장애물 감지)
- **자이로스코프** (방향 센서)
- **엔코더** (정밀한 위치 추적)

## 소프트웨어 구조

```
SCV_Robot/
├── SCV_Robot.ino          # 메인 프로그램 (로봇 제어 로직)
├── motorControl.h/cpp     # 모터 제어 모듈
├── beaconManager.h/cpp    # 비콘 위치 인식 모듈
├── pathfinder.h/cpp       # A* 경로 탐색 모듈
└── communication.h/cpp    # WiFi 통신 모듈 (API 서버)
```

## 설치 및 설정

### 1. Arduino IDE 설정
1. Arduino IDE 설치
2. Arduino R4 보드 패키지 설치
3. 필요한 라이브러리 설치:
   - `WiFi` (기본 포함)
   - `BLEDevice` (기본 포함)
   - `ArduinoJson` (라이브러리 매니저에서 설치)

### 2. 하드웨어 연결
```
모터 연결:
- LEFT_MOTOR_PWM_PIN: 9
- LEFT_MOTOR_DIR_PIN: 8
- RIGHT_MOTOR_PWM_PIN: 10
- RIGHT_MOTOR_DIR_PIN: 11

비콘 설정:
- 비콘 1: (0, 0) 위치
- 비콘 2: (10, 0) 위치
- 비콘 3: (5, 10) 위치
```

### 3. 설정 파일 수정
`SCV_Robot.ino`에서 다음 설정을 수정하세요:

```cpp
// WiFi 설정
const char* WIFI_SSID = "your_wifi_ssid";
const char* WIFI_PASSWORD = "your_wifi_password";

// 비콘 주소 (beaconManager.h에서 수정)
String beaconAddresses[NUM_BEACONS] = {
    "BE:AC:ON:01:02:03",  // 실제 비콘 주소로 변경
    "BE:AC:ON:04:05:06",
    "BE:AC:ON:07:08:09"
};
```

## 사용법

### 1. 로봇 실행
1. Arduino IDE에서 `SCV_Robot.ino` 업로드
2. 시리얼 모니터로 초기화 과정 확인
3. WiFi 연결 및 IP 주소 확인

### 2. API 서버 접속
로봇의 IP 주소로 HTTP API 호출:
```
http://[로봇_IP_주소]/status
```

### 3. 제어 명령
- **위치 이동**: POST `/move` 엔드포인트로 좌표 전송
- **긴급 정지**: POST `/emergency-stop` 엔드포인트 호출
- **속도 조절**: POST `/set-speed` 엔드포인트로 속도 설정
- **상태 확인**: GET `/status` 엔드포인트로 상태 조회

## API 명세

### 이동 명령
```json
POST /move
Content-Type: application/json

{
    "command": "move_to_position",
    "x": 5.0,
    "y": 3.0,
    "speed": 200
}

Response:
{
    "success": true,
    "message": "Command executed",
    "command": "move_to_position"
}
```

### 상태 조회
```json
GET /status

Response:
{
    "currentX": 2.5,
    "currentY": 1.8,
    "targetX": 5.0,
    "targetY": 3.0,
    "isMoving": true,
    "currentSpeed": 200,
    "batteryLevel": 85.5,
    "lastError": ""
}
```

### 긴급 정지
```json
POST /emergency-stop
Content-Type: application/json

{
    "command": "emergency_stop"
}

Response:
{
    "success": true,
    "message": "Emergency stop activated"
}
```

### 속도 설정
```json
POST /set-speed
Content-Type: application/json

{
    "speed": 150
}

Response:
{
    "success": true,
    "message": "Speed updated"
}
```

## 개발 가이드

### 새로운 기능 추가
1. 해당 모듈의 `.h` 파일에 인터페이스 정의
2. `.cpp` 파일에 구현 작성
3. `SCV_Robot.ino`에서 모듈 통합
4. 필요시 communication 모듈에 API 엔드포인트 추가

### Communication 모듈 확장
```cpp
// 새로운 명령 타입 추가
enum CommandType {
    // ... 기존 명령들
    CMD_NEW_FEATURE,
    CMD_UNKNOWN
};

// API 엔드포인트 추가
void Communication::handleNewFeature() {
    // 새로운 기능 처리
}

// 메인 컨트롤러에서 처리
void handleCommand(const MoveCommand& command) {
    switch(command.type) {
        // ... 기존 케이스들
        case CMD_NEW_FEATURE:
            // 새로운 기능 실행
            break;
    }
}
```

### 디버깅
- 시리얼 모니터를 통한 로그 확인
- 각 모듈별 상세한 디버그 메시지 제공
- API 응답을 통한 상태 확인

## 문제 해결

### 일반적인 문제
1. **WiFi 연결 실패**: SSID/비밀번호 확인
2. **비콘 인식 안됨**: 비콘 주소 및 거리 확인
3. **모터 동작 안됨**: 핀 연결 및 전원 공급 확인
4. **경로 찾기 실패**: 장애물 설정 및 그리드 크기 확인
5. **API 응답 없음**: 콜백 함수 설정 확인

### 성능 최적화
- 비콘 스캔 주기 조정
- 경로 탐색 그리드 크기 최적화
- 모터 속도 제한 설정
- 로깅 레벨 조정

## Next.js 앱 연동

### 프론트엔드 개발
```javascript
// 로봇 제어 예시
const moveRobot = async (x, y, speed) => {
    const response = await fetch('http://[로봇_IP]/move', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            command: 'move_to_position',
            x: x,
            y: y,
            speed: speed
        })
    });
    return response.json();
};

// 상태 조회 예시
const getRobotStatus = async () => {
    const response = await fetch('http://[로봇_IP]/status');
    return response.json();
};
```

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여하기

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 연락처

프로젝트 관련 문의사항이 있으시면 이슈를 등록해 주세요.
