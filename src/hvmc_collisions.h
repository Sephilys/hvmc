#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"


struct RigidBody;

struct Ray
{
    Ray(vec2 pos, vec2 dir) : pos(pos), dir(dir)
    {
        invDir.x = dir.x ? 1.0 / dir.x : INFINITY;
        invDir.y = dir.y ? 1.0 / dir.y : INFINITY;
    }
    vec2 pos;
    vec2 dir;
    vec2 invDir;
};

struct CollisionInfo{

    RigidBody* rb1;
    RigidBody* rb2;
    vec2 contactNormal; //normal au contact
    double interPenetrationDistance;    //distance interpenetration
    vec2 contactPosition;   //position du point (ses coordonnées en fait) au contact tu vois

    CollisionInfo() {}

    CollisionInfo(RigidBody* rb1, RigidBody* rb2, vec2 contactNormal, double penetration, vec2 contactPoint){
        this->rb1 = rb1;
        this->rb2 = rb2;
        this->contactNormal = contactNormal;
        this->interPenetrationDistance = penetration;
        this->contactPosition = contactPoint;
    }

    void Solve() const;
    void PositionCorrection() const;
    bool debug;
};

std::ostream& operator<< (std::ostream& os, vec2 v);

bool continuousDetectionBox(RigidBody& box, RigidBody& body);
bool CollideCircleCircle( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info);
bool CollideBoxBox( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info);
bool Collide( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info );




#endif
