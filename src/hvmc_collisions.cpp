#include "hvmc_collisions.h"
#include "hvmc_physics.h"



/**
* petit calcul alakon
**/

double norm(vec2 vect){
    return sqrt (vect.x * vect.x + vect.y * vect.y);
}


/**
 *out : CollisionInfo info
 **/
bool CollideCircleCircle( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info){

    double dist = sqrt ((rb1->position.x -rb2->position.x) * (rb1->position.x -rb2->position.x) + (rb1->position.y -rb2->position.y) * (rb1->position.y -rb2->position.y) );

    if (dist < rb1->collider.radius +  rb2->collider.radius ){

        info.contactNormal = Normalize(rb2->position - rb1->position);
        info.interPenetrationDistance = rb2->collider.radius + rb1->collider.radius - norm(rb2->position - rb1->position);
        info.contactPosition = rb1->position + rb1->collider.radius * info.contactNormal;
        info.rb1 = rb1;
        info.rb2 = rb2;
        return true;
    }
    return false;
}


///**
// *out : CollisionInfo info
// **/
//bool CollideBoxBox( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info){

//    double dist = sqrt ((rb1->position.x -rb2->position.x) * (rb1->position.x -rb2->position.x) + (rb1->position.y -rb2->position.y) * (rb1->position.y -rb2->position.y) );

//    if (dist < rb1->collider.radius +  rb2->collider.radius ){

//        info.contactNormal = Normalize(rb2->position - rb1->position);
//        info.interPenetrationDistance = rb2->collider.radius + rb1->collider.radius - norm(rb2->position - rb1->position);
//        info.contactPosition = rb1->position + rb1->collider.radius * info.contactNormal;
//        info.rb1 = rb1;
//        info.rb2 = rb2;
//        return true;
//    }
//    return false;
//}



/**
 *out : CollisionInfo info
 **/

bool Collide( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info ){


    if (( rb1->collider.type == RIGID_BODY_SPHERE) && (rb2->collider.type == RIGID_BODY_SPHERE)){

        if (CollideCircleCircle(rb1, rb2, info)){

            info.rb1 = rb1;
            info.rb2 = rb2;
            return true;
        }
        return false;
   }
    return false;

}


