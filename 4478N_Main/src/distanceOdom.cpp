#include "distanceOdom.h"
#include "devices.h"
#include "lemlib/api.hpp"
#include <cmath>

using namespace pros;
using namespace lemlib;

// ---------- distance odom ----------
// 140x140 field centered on 0,0 -- walls at +/-70. heading is lemlib theta (+Y is 0)
// body +x right +y fwd. calcs figure which wall youre facing and snap x or y

// sensor to center along each face (in) -- should match header constants
double frontOffset = 2.0;
double backOffset = 2.0;
double leftOffset = 5;
double rightOffset = 5;
// *EdgeOffsets: along bumper (F/B + = right +x, L/R + = forward +y). Cardinal*sin(d) added to perp distance after perpAlongNormal.
double frontEdgeOffset = 4.5;
double backEdgeOffset = 5;
double leftEdgeOffset = -3;
double rightEdgeOffset = 0;
const double fieldHalfSize = 71.0; // in
static double theta = 0.0;
static double x = chassis.getPose().x;
static double y = chassis.getPose().y;

// misc helpers
double mmToInches(double mm) { return mm / 25.4; }
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
double norm360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0) {
        deg += 360.0;
    }
    return deg;
}
double smallestAngleDiffDeg(double aDeg, double bDeg) {
    double d = std::fabs(aDeg - bDeg);
    if (d > 180.0) {
        d = 360.0 - d;
    }
    return d;
}
// slant range * cos to get perp dist to wall
double perpAlongNormal(double rangeInches, double boreDeg, double normalDeg) {
    const double d = smallestAngleDiffDeg(norm360(boreDeg), norm360(normalDeg));
    return rangeInches * std::cos(degreesToRadians(d));
}
// ±inches from heading vs cardinals; scaled by sin(d), added to perpendicular (after perpAlongNormal).
static double cardinalEdgeTerm(double headingDeg, double signedEdge) {
    const double t = norm360(headingDeg);
    if (t < 45) return -signedEdge;
    if (t <= 90) return signedEdge;
    if (t <= 135) return -signedEdge;
    if (t <= 180) return signedEdge;
    if (t <= 225) return -signedEdge;
    if (t <= 270) return signedEdge;
    if (t <= 315) return -signedEdge;
    return signedEdge;
}

static double edgePerpCorr(double boreDeg, double normalDeg, double headingDeg, double signedEdge) {
    const double d = smallestAngleDiffDeg(norm360(boreDeg), norm360(normalDeg));
    return cardinalEdgeTerm(headingDeg, signedEdge) * std::sin(degreesToRadians(d));
}

// one calc per sensor - branches are which wall based on heading
void calcLeft(){
    const double boreL = theta - 90.0;
    const double leftReading = mmToInches(leftDistance->get()) + leftOffset;

    if(theta <= 45 || theta > 315){
        x = -fieldHalfSize + perpAlongNormal(leftReading, boreL, 270.0) +
            edgePerpCorr(boreL, 270.0, theta, leftEdgeOffset);
    }
    else if(theta >= 135 && theta <= 225){
        x = fieldHalfSize - perpAlongNormal(leftReading, boreL, 90.0) -
            edgePerpCorr(boreL, 90.0, theta, leftEdgeOffset);
    }
    else if (theta >= 45 && theta <= 135){
        y = fieldHalfSize - perpAlongNormal(leftReading, boreL, 0.0) -
            edgePerpCorr(boreL, 0.0, theta, leftEdgeOffset);
    }
    else if(theta >= 225 && theta <= 315){
        y = -fieldHalfSize + perpAlongNormal(leftReading, boreL, 180.0) +
            edgePerpCorr(boreL, 180.0, theta, leftEdgeOffset);
    }
} 


void calcRight(){
    const double boreR = theta + 90.0; 
    const double rightReading = mmToInches(rightDistance->get()) + rightOffset;

    if(theta <= 45 || theta > 315){
        x = fieldHalfSize - perpAlongNormal(rightReading, boreR, 90.0) -
            edgePerpCorr(boreR, 90.0, theta, rightEdgeOffset);
    }
    else if(theta >= 135 && theta <= 225){
        x = -fieldHalfSize + perpAlongNormal(rightReading, boreR, 270.0) +
            edgePerpCorr(boreR, 270.0, theta, rightEdgeOffset);
    }
    else if (theta >= 45 && theta <= 135){
        y = -fieldHalfSize + perpAlongNormal(rightReading, boreR, 180.0) +
            edgePerpCorr(boreR, 180.0, theta, rightEdgeOffset);
    }
    else if(theta >= 225 && theta <= 315){
        y = fieldHalfSize - perpAlongNormal(rightReading, boreR, 0.0) -
            edgePerpCorr(boreR, 0.0, theta, rightEdgeOffset);
    }

}

void calcFront(){
    const double boreF = theta;
    const double frontReading = mmToInches(frontDistance->get()) + frontOffset;

    if(theta >= 45 && theta <= 135){
        x = fieldHalfSize - perpAlongNormal(frontReading, boreF, 90.0) -
            edgePerpCorr(boreF, 90.0, theta, frontEdgeOffset);
    }
    else if(theta >= 225 && theta <= 315){
        x = -fieldHalfSize + perpAlongNormal(frontReading, boreF, 270.0) +
            edgePerpCorr(boreF, 270.0, theta, frontEdgeOffset);
    }
    else if (theta <= 45 || theta > 315){
        y = fieldHalfSize - perpAlongNormal(frontReading, boreF, 0.0) -
            edgePerpCorr(boreF, 0.0, theta, frontEdgeOffset);
    }
    else if(theta >= 135 && theta <= 225){
        y = -fieldHalfSize + perpAlongNormal(frontReading, boreF, 180.0) +
            edgePerpCorr(boreF, 180.0, theta, frontEdgeOffset);
    }
}

void calcBack(){
    const double boreB = theta + 180.0;
    const double backReading = mmToInches(backDistancePtr->get()) + backOffset;

    if(theta >= 45 && theta <= 135){
        x = -fieldHalfSize + perpAlongNormal(backReading, boreB, 270.0) +
            edgePerpCorr(boreB, 270.0, theta, backEdgeOffset);
    }
    else if(theta >= 225 && theta <= 315){
        x = fieldHalfSize - perpAlongNormal(backReading, boreB, 90.0) -
            edgePerpCorr(boreB, 90.0, theta, backEdgeOffset);
    }
    else if (theta <= 45 || theta > 315){
        y = -fieldHalfSize + perpAlongNormal(backReading, boreB, 180.0) +
            edgePerpCorr(boreB, 180.0, theta, backEdgeOffset);
    }
    else if(theta >= 135 && theta <= 225){
        y = fieldHalfSize - perpAlongNormal(backReading, boreB, 0.0) -
            edgePerpCorr(boreB, 0.0, theta, backEdgeOffset);
    }
}

// call from auton or tester -- bools = which sensors to trust, sets pose from walls
void resetOdom(bool Front, bool Back, bool Left, bool Right){
    // LemLib heading can be negative or outside [0,360); wall branches require 0–360.
    theta = norm360(chassis.getPose().theta);
    x = chassis.getPose().x;
    y = chassis.getPose().y;

    if(Front){
        calcFront();
    }
    if(Back){
        calcBack(); 
    }
    if(Left){
        calcLeft();
    }
    if(Right){
        calcRight();
    }

    chassis.setPose(x, y, theta);
}