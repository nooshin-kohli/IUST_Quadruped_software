/*
* Created by aligolestaneh on Aug 27 2022
*
* @brief Utility function to build a IUST Quadruped object
*
*
* This file builds a model of The IUST robot.
* The inertia parameters of all bodies are determined from CAD by mohammadjabarzadeh.
* Modified with new body: AminGhanbarzadeh
*/

#ifndef IUST_CHEETAH_SOFTWARE_IUST_H
#define IUST_CHEETAH_SOFTWARE_IUST_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of IUST robot
 */
template<typename T>
Quadruped<T> buildIUST()
{
    Quadruped<T> iust;
    iust._robotType = RobotType::IUST;

    //Mass(kg)
    iust._bodyMass = 3.94; //Done
    iust._abadMass = 0.650; //Done
    iust._hipMass = 1.154; //Done
    iust._kneeMass = 0.209; //Done
    iust._rotorMass = 0.084; //NI

    //Dimentions(m)
    iust._bodyLength = 0.46; //0.269 * //Done
    iust._bodyWidth = 0.11; //0.19054 * //Done
    iust._bodyHeight = 0.095; //Done

    //Gear ratios
    iust._abadGearRatio = 6;
    iust._hipGearRatio = 6;
    iust._kneeGearRatio = 6;

    //Link lengths(m)
    iust._abadLinkLength = 0.0576; //Done
    iust._hipLinkLength = 0.22886; //Done 
    iust._kneeLinkY_offset = 0.07125; //NI?
    iust._kneeLinkLength = 0.240; //Done
    iust._maxLegLength = 0.468; //Done
    iust._hipRotorLocationYOffset = 0.0527; //NI?

    //Motor properties
    iust._motorTauMax = 8.f;
    iust._batteryV = 24;
    iust._motorKT = 0.08;   //(MiLab Team: this is flux linkage * pole pairs)
    iust._motorR = 0.13;
    iust._jointDamping = 0.02;
    iust._jointDryFriction = 0.4;

    //Rotor inertia (if the rotor is oriented so it spins around the z-axis)
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 53, 0, 0, 0, 53, 0, 0, 0, 105;
    rotorRotationalInertiaZ = rotorRotationalInertiaZ * 1e-6;

    Mat3 <T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3 <T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3 <T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3 <T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

    //Spatial inertia for abad
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 712.408, -3.535, -5.317, -3.535, 1095.432, 13.78, -5.317, 13.78, 772.605;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(-0.004444, -0.000541, -0.000536);  // LEFT
    SpatialInertia<T> abadInertia(iust._abadMass, abadCOM, abadRotationalInertia);

    //Spatial inertia for hip
    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 17479.820, -9.63, 1236.601, -9.63, 17229.4, -20.313, 1236.601, -20.313, 2982.14;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(-0.004928, -0.022568, -0.039632);
    SpatialInertia<T> hipInertia(iust._hipMass, hipCOM, hipRotationalInertia);

    //Spatial inertia for knee
    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 7880.558, -0.083, -15.380, -0.083, 7917.82, -0.27, -15.38, -0.27, 71.290;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    //kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(-0.000838, -0.000038, -0.157249);
    SpatialInertia<T> kneeInertia(iust._kneeMass, kneeCOM, kneeRotationalInertiaRotated);

    //Rotor inertia (x-axis and y-axis)
    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(iust._rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(iust._rotorMass, rotorCOM, rotorRotationalInertiaY);

    //Body inertia
    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 84746.639, -505.566, -1520.915, -505.566, 460537.565, 143.817, -1520.915, 143.817, 487181.361;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(-0.004017, -0.000209, -0.005174);
    Vec3<T> bodyDims(iust._bodyLength, iust._bodyWidth, iust._bodyHeight);
    SpatialInertia<T> bodyInertia(iust._bodyMass, bodyCOM, rotInertiaOfBox(iust._bodyMass, bodyDims));
    
    //Adjust IUST inertias
    iust._abadInertia = abadInertia;
    iust._hipInertia = hipInertia;
    iust._kneeInertia = kneeInertia;
    iust._abadRotorInertia = rotorInertiaX;
    iust._hipRotorInertia = rotorInertiaY;
    iust._kneeRotorInertia = rotorInertiaY;
    iust._bodyInertia = bodyInertia;

    //Locations
    iust._abadRotorLocation = Vec3<T>(iust._bodyLength, iust._bodyWidth, 0) * 0.5;
    iust._abadLocation = Vec3<T>(iust._bodyLength, iust._bodyWidth, 0) * 0.5;
    iust._hipLocation = Vec3<T>(0, iust._abadLinkLength, 0);
    iust._hipRotorLocation = Vec3<T>(0, iust._abadLinkLength - iust._hipRotorLocationYOffset, 0);
    iust._kneeLocation = Vec3<T>(0, 0, -iust._hipLinkLength);
    iust._kneeRotorLocation = Vec3<T>(0, 0, 0);

    return iust;
}

#endif //IUST_H