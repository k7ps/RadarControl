#include "radar_control/calculations.h"

#include <gtest/gtest.h>


const double VIEW_ANGLE = 60;
const double MARGIN = 5;


TEST(CalculateRadarAngleOneTarget, TargetAlreadySet) {
    double targetAngle = 90;
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(30, targetAngle, 66, 100, VIEW_ANGLE, MARGIN), targetAngle);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(98, targetAngle, 115, 70, VIEW_ANGLE, MARGIN), targetAngle);
}

TEST(CalculateRadarAngleOneTarget, BothPointsInside) {
    double currAngle = 90;
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(currAngle, -1, 66, 100, VIEW_ANGLE, MARGIN), currAngle);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(currAngle, -1, 115, 70, VIEW_ANGLE, MARGIN), currAngle);
}

TEST(CalculateRadarAngleOneTarget, EntryOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 130, 100, VIEW_ANGLE, MARGIN), 105);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 20, 69, VIEW_ANGLE, MARGIN), 45);
}

TEST(CalculateRadarAngleOneTarget, MeetingOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 100, 140, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 70, 30, VIEW_ANGLE, MARGIN), 55);
}

TEST(CalculateRadarAngleOneTarget, BothCantBeInside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 100, 160, VIEW_ANGLE, MARGIN), 125);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 140, 20, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 110, 30, VIEW_ANGLE, MARGIN), 85);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 80, 170, VIEW_ANGLE, MARGIN), 105);
}

TEST(CalculateRadarAngleOneTarget, BothOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 130, 140, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 170, 120, VIEW_ANGLE, MARGIN), 145);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 10, 30, VIEW_ANGLE, MARGIN), 35);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 50, 10, VIEW_ANGLE, MARGIN), 35);
}

TEST(CalculateRadarAngleOneTarget, Random) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 171, 89, VIEW_ANGLE, MARGIN), 146);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(90, -1, 47, 36, VIEW_ANGLE, MARGIN), 61);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(30, -1, 54, 63, VIEW_ANGLE, MARGIN), 38);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(112, -1, 73, 73, VIEW_ANGLE, MARGIN), 98);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(151, -1, 163, 122, VIEW_ANGLE, MARGIN), 147);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleOneTarget(11, 140, 163, 126, VIEW_ANGLE, MARGIN), 140);
}

TEST(CalculateRadarAngleMultiTarget, Random) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngleMultiTarget(30, 90, {66, 70, 100}, VIEW_ANGLE, MARGIN), 90);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleMultiTarget(90, -1, {66, 102}, VIEW_ANGLE, MARGIN), 90);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleMultiTarget(90, -1, {120, 100, 140}, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleMultiTarget(90, -1, {170, 120}, VIEW_ANGLE, MARGIN), 145);
    EXPECT_DOUBLE_EQ(CalculateRadarAngleMultiTarget(90, -1, {10, 40, 30}, VIEW_ANGLE, MARGIN), 35);
}
