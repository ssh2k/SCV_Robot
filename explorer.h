#pragma once

#include <vector>

namespace Explorer {

struct Point {
	int x;
	int y;
	bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

enum class Direction { Up, Right, Down, Left };

// Grid utilities
void initializeGrid(int defaultValue = 0);
int width();
int height();
int getCell(int x, int y);
void setCell(int x, int y, int value);

// Beacons
void setBeacons(const std::vector<Point>& newBeacons);
std::vector<int> computePathLengthsToBeacons(const Point& start, bool allowUnknown = false);

// Export grid for Pathfinder (0=free, 1=blocked[wall/unknown])
std::vector<std::vector<int>> exportGridForPathfinder();

// Robot pose helpers
Direction getDirection();
void setDirection(Direction d);

// Exploration and path finding
void exploreMap(const Point& start, int maxSteps = 1000);
std::vector<Point> aStar(const Point& start, const Point& goal, bool allowUnknown = false);

} // namespace Explorer


