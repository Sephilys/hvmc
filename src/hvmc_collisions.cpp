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


/**
 *out : CollisionInfo info
 **/

bool CollideBoxBox( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info){

    double overlapX, overlapY;

    double rb1XMin = rb1->position.x;
    double rb1XMax = rb1->position.x +rb1->collider.dims.x;
    double rb1YMin = rb1->position.y;
    double rb1YMax = rb1->position.y +rb1->collider.dims.y;

    double rb2XMin = rb2->position.x;
    double rb2XMax = rb2->position.x +rb2->collider.dims.x;
    double rb2YMin = rb2->position.y;
    double rb2YMax = rb2->position.y +rb2->collider.dims.y;

    bool rb1sup;//rb1 au dessus de rb2

    /*test d'un overlap en abscisse*/

    if ((rb1XMin > rb2XMin) && (rb1XMin<rb2XMax)){
        overlapX = abs(rb1XMin - rb2XMax);
    }

    else if ((rb2XMin > rb1XMin) && (rb2XMin<rb1XMax)){
        overlapX = abs(rb2XMin - rb1XMax);
    }

    else return false;


    /*test d'un overlap en ordonnée*/

    if ((rb1YMin > rb2YMin) && (rb1YMin<rb2YMax)){
        overlapY = abs(rb1YMin - rb2YMax);
        rb1sup = true;
    }

    else if ((rb2YMin > rb1YMin) && (rb2YMin<rb1YMax)){
        overlapY = abs(rb2YMin - rb1YMax);
        rb1sup = false;
    }

    else return false;

    /// à partir de ici il y a bien collision




    if (overlapX < overlapY){

        info.interPenetrationDistance = overlapX;

        if (rb1sup)
            info.contactNormal = vec2{-1,0};
        else
            info.contactNormal = vec2{1,0};
    }else{

        info.interPenetrationDistance = overlapY;

        if (rb1sup)
            info.contactNormal = vec2{0,-1};
        else
            info.contactNormal = vec2{0,1};
    }

    info.rb1 = rb1;
    info.rb2 = rb2;


    /*provisoire*/
    info.contactPosition = rb1->position + rb1->collider.dims.x * info.contactNormal;

    return true;

}


void CollisionInfo::Solve() const
{
//    rb1->ApplyForce(vec2{10, 0});
//    float m_a = rb1->m;
//    float m_b = rb2->m;
//    vec2 v_a = rb1->velocity;
//    vec2 v_b = rb2->velocity;

//    float e = -(m_a * a_a - m_b * a_b) / (m_a * v_a - m_b * v_b);
//    float e = 1.f;
//    float J = (-(1 + e) * Dot(v_rel, n)) * ;
    rb1->ApplyImpulse(-10 * contactNormal, vec2{0, 0});
//    rb2->ApplyImpulse();
}


/**
 *out : CollisionInfo info
 **/
/*
bool Collide( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info ){


    if (( rb1->collider.type == RIGID_BODY_SPHERE) && (rb2->collider.type == RIGID_BODY_SPHERE)){

        if (CollideCircleCircle(rb1, rb2, info)) {
            std::cout<<"yop"<<std::endl;
            return true;
        }
        return false;
    }
    else if (( rb1->collider.type == RIGID_BODY_BOX) && (rb2->collider.type == RIGID_BODY_BOX)){
        std::cout<<"coucou"<<std::endl;
        if (CollideBoxBox(rb1, rb2, info)){
            std::cout<<"ça touche"<<std::endl;
            return true;
        }
        return false;
    }
    return false;

}

*/

bool Collide( RigidBody* a, RigidBody* b, CollisionInfo& info ) {

    if ( a->collider.type == RIGID_BODY_BOX )
    {
        if ( b->collider.type == RIGID_BODY_BOX )
            return CollideBoxBox( a, b, info );
        //else if ( b->collider.type == RIGID_BODY_SPHERE )
          //  return CollideBoxSphere( a, b, info );
    }

    else if ( a->collider.type == RIGID_BODY_SPHERE )
    {
        if ( b->collider.type == RIGID_BODY_SPHERE )
            return CollideCircleCircle( a, b, info );
        //else if ( b->collider.type == RIGID_BODY_BOX )
          //  return CollideSphereBox( a, b, info );
    }

    // Should not get there
    return false;
}

