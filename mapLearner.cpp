#include "mapLearner.h"
#include <Arduino.h>
#include <math.h>

MapLearner::MapLearner(Pathfinder* pathfinder, int gridWidth, int gridHeight)
    : pathfinder(pathfinder), gridWidth(gridWidth), gridHeight(gridHeight) {
    learnedGrid = new bool*[gridHeight];
    for (int y = 0; y < gridHeight; y++) {
        learnedGrid[y] = new bool[gridWidth];
        for (int x = 0; x < gridWidth; x++) {
            learnedGrid[y][x] = false;
        }
    }
}

void MapLearner::begin() {
    clearLearnedGrid();
}

void MapLearner::clearLearnedGrid() {
    for (int y = 0; y < gridHeight; y++) {
        for (int x = 0; x < gridWidth; x++) {
            learnedGrid[y][x] = false;
        }
    }
}

void MapLearner::learnMap() {
    for (int x = 0; x < gridWidth; x++) {
        for (int y = 0; y < gridHeight; y++) {
            scanAtGrid(x, y);
        }
    }
}

void MapLearner::scanAtGrid(int gridX, int gridY) {
    // 4방향(0, 90, 180, 270도) 스캔 예시
    const int angles[4] = {0, 90, 180, 270};
    const float OBSTACLE_THRESHOLD = 0.5; // 0.5m 이내 장애물 감지
    for (int i = 0; i < 4; i++) {
        float dist = getUltrasonicDistance(angles[i]);
        if (dist > 0 && dist < OBSTACLE_THRESHOLD) {
            // 장애물 위치 계산 (간단화)
            int obsX = gridX + round(cos(angles[i] * DEG_TO_RAD));
            int obsY = gridY + round(sin(angles[i] * DEG_TO_RAD));
            if (obsX >= 0 && obsX < gridWidth && obsY >= 0 && obsY < gridHeight) {
                markObstacle(obsX, obsY);
            }
        }
    }
}

float MapLearner::getUltrasonicDistance(int angleDeg) {
    // TODO: 실제 초음파 센서 연동 필요
    // 현재는 더미 값 반환 (1.0m = 장애물 없음)
    return 1.0;
}

void MapLearner::markObstacle(int gridX, int gridY) {
    learnedGrid[gridY][gridX] = true;
}

void MapLearner::applyLearnedMap() {
    for (int x = 0; x < gridWidth; x++) {
        for (int y = 0; y < gridHeight; y++) {
            pathfinder->setObstacle(x, y, learnedGrid[y][x]);
        }
    }
} 