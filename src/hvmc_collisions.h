#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"

struct RigidBody;

struct CollisionInfo{

    RigidBody* rb1;
    RigidBody* rb2;
    vec2 contactNormal; //normal au contact
    double interPenetrationDistance;    //distance interpenetration
    vec2 contactPosition;   //position du point (ses coordonnées en fait) au contact tu vois


    CollisionInfo(RigidBody* rb1, RigidBody* rb2){
        this->rb1 = rb1;
        this->rb2 = rb2;
    }





};





#endif
