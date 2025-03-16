#include "radar_control/lib/calculations.h"

#include <gtest/gtest.h>


const double VIEW_ANGLE = 60;
const double MARGIN = 5;


TEST(CalculateRadarAngle1Target, BothPointsInside) {
    double currAngle = 90;

    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(currAngle, 66, 100, VIEW_ANGLE, MARGIN), currAngle);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(currAngle, 115, 70, VIEW_ANGLE, MARGIN), currAngle);
}

TEST(CalculateRadarAngle1Target, EntryOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 130, 100, VIEW_ANGLE, MARGIN), 105);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 20, 69, VIEW_ANGLE, MARGIN), 45);
}

TEST(CalculateRadarAngle1Target, MeetingOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 100, 140, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 70, 30, VIEW_ANGLE, MARGIN), 55);
}

TEST(CalculateRadarAngle1Target, BothCantBeInside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 100, 160, VIEW_ANGLE, MARGIN), 125);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 140, 20, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 110, 30, VIEW_ANGLE, MARGIN), 85);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 80, 170, VIEW_ANGLE, MARGIN), 105);
}

TEST(CalculateRadarAngle1Target, BothOutside) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 130, 140, VIEW_ANGLE, MARGIN), 115);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 170, 120, VIEW_ANGLE, MARGIN), 145);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 10, 30, VIEW_ANGLE, MARGIN), 35);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 50, 10, VIEW_ANGLE, MARGIN), 35);
}

TEST(CalculateRadarAngle1Target, Random) {
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 171, 89, VIEW_ANGLE, MARGIN), 146);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(90, 47, 36, VIEW_ANGLE, MARGIN), 61);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(30, 54, 63, VIEW_ANGLE, MARGIN), 38);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(112, 73, 73, VIEW_ANGLE, MARGIN), 98);
    EXPECT_DOUBLE_EQ(CalculateRadarAngle1Target(151, 163, 122, VIEW_ANGLE, MARGIN), 147);
}
