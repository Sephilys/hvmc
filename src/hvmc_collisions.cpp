#include "hvmc_collisions.h"
#include "hvmc_physics.h"



/**
* return : norme du vecteur
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

    double rb1XMin = rb1->position.x - rb1->collider.dims.x/2;
    double rb1XMax = rb1->position.x + rb1->collider.dims.x/2;
    double rb1YMin = rb1->position.y - rb1->collider.dims.y/2;
    double rb1YMax = rb1->position.y + rb1->collider.dims.y/2;

    double rb2XMin = rb2->position.x - rb2->collider.dims.x/2;
    double rb2XMax = rb2->position.x + rb2->collider.dims.x/2;
    double rb2YMin = rb2->position.y - rb2->collider.dims.y/2;
    double rb2YMax = rb2->position.y + rb2->collider.dims.y/2;

    bool rb1sup;//rb1 au dessus de rb2

    /*test d'un overlap en abscisse*/

    if ((rb1XMin >= rb2XMin) && (rb1XMin <= rb2XMax)){
        overlapX = abs(rb1XMin - rb2XMax);
    }

    else if ((rb2XMin >= rb1XMin) && (rb2XMin <= rb1XMax)){
        overlapX = abs(rb2XMin - rb1XMax);
    }

    else return false;


    /*test d'un overlap en ordonnée*/

    if ((rb1YMin >= rb2YMin) && (rb1YMin <= rb2YMax)){
        overlapY = abs(rb1YMin - rb2YMax);
        rb1sup = true;
    }

    else if ((rb2YMin >= rb1YMin) && (rb2YMin <= rb1YMax)){
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
    //info.contactPosition = rb1->position + rb1->collider.dims.x * info.contactNormal;

    return true;

}



bool CollideCircleBox( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info){

    double overlapX, overlapY;

    double rb1XMin = rb1->position.x - rb1->collider.radius;
    double rb1XMax = rb1->position.x + rb1->collider.radius;
    double rb1YMin = rb1->position.y - rb1->collider.radius;
    double rb1YMax = rb1->position.y + rb1->collider.radius;

    double rb2XMin = rb2->position.x - rb2->collider.dims.x /2;
    double rb2XMax = rb2->position.x + rb2->collider.dims.x /2;
    double rb2YMin = rb2->position.y - rb2->collider.dims.y /2;
    double rb2YMax = rb2->position.y + rb2->collider.dims.y /2;

    bool rb1sup;//rb1 au dessus de rb2

    /*test d'un overlap en abscisse*/

    if ((rb1XMin >= rb2XMin) && (rb1XMin<=rb2XMax)){
        overlapX = abs(rb1XMin - rb2XMax);
    }

    else if ((rb2XMin >= rb1XMin) && (rb2XMin<=rb1XMax)){
        overlapX = abs(rb2XMin - rb1XMax);
    }

    else return false;


    /*test d'un overlap en ordonnée*/

    if ((rb1YMin >= rb2YMin) && (rb1YMin<=rb2YMax)){
        overlapY = abs(rb1YMin - rb2YMax);
        rb1sup = true;
    }

    else if ((rb2YMin >= rb1YMin) && (rb2YMin<=rb1YMax)){
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



    //info.contactPosition = rb1->position + rb1->collider.radius * info.contactNormal;

    return true;

}



bool CollideBoxCircle( RigidBody* rb1, RigidBody* rb2, CollisionInfo &info){

    double overlapX, overlapY;

    double rb1XMin = rb1->position.x - rb1->collider.dims.x /2;
    double rb1XMax = rb1->position.x + rb1->collider.dims.x /2;
    double rb1YMin = rb1->position.y - rb1->collider.dims.y /2;
    double rb1YMax = rb1->position.y + rb1->collider.dims.y /2;

    double rb2XMin = rb2->position.x - rb2->collider.radius;
    double rb2XMax = rb2->position.x + rb2->collider.radius;
    double rb2YMin = rb2->position.y - rb2->collider.radius;
    double rb2YMax = rb2->position.y + rb2->collider.radius;

    bool rb1sup;//rb1 au dessus de rb2

    /*test d'un overlap en abscisse*/

    if ((rb1XMin >= rb2XMin) && (rb1XMin<=rb2XMax)){
        overlapX = abs(rb1XMin - rb2XMax);
    }

    else if ((rb2XMin >= rb1XMin) && (rb2XMin<=rb1XMax)){
        overlapX = abs(rb2XMin - rb1XMax);
    }

    else return false;


    /*test d'un overlap en ordonnée*/

    if ((rb1YMin >= rb2YMin) && (rb1YMin<=rb2YMax)){
        overlapY = abs(rb1YMin - rb2YMax);
        rb1sup = true;
    }

    else if ((rb2YMin >= rb1YMin) && (rb2YMin<=rb1YMax)){
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



    //info.contactPosition = rb1->position + rb1->collider.radius * info.contactNormal;

    return true;

}

void CollisionInfo::Solve() const
{

    // Renommage des données
//    f32 m_a = rb1->m;
//    f32 m_b = rb2->m;
    f32 im_a = rb1->im;
    f32 im_b = rb2->im;
    f32 iI_a = rb1->iI;
    f32 iI_b = rb2->iI;
    vec2 v_a = rb1->velocity;
    vec2 v_b = rb2->velocity;
    f32 w_a = rb1->angularVelocity;
    f32 w_b = rb2->angularVelocity;

    vec2 n = contactNormal; //

    vec2 r_a =  rb1->collider.radius * n; //
    vec2 r_b = -rb2->collider.radius * n; //
    vec2 v_rel = (v_b + Cross(w_b, r_b)) - (v_a + Cross(w_a, r_a)); //

//    f32 e = -(m_a * a_a - m_b * a_b) / (m_a * v_a - m_b * v_b);
    f32 e = 0.8f;

    f32 J = (-(1 + e) * Dot(v_rel, n)) / (im_a + im_b + iI_a * Cross(r_a, n) + iI_b * Cross(r_b, n)); //

    vec2 j_a = -J * n;
    vec2 j_b =  J * n;

    if (Dot(v_rel,n) < 0){
        rb1->ApplyImpulse(j_a, contactPosition);
        rb2->ApplyImpulse(j_b, contactPosition);
    }
}



/**
 *out : CollisionInfo info
 **/
bool Collide( RigidBody* a, RigidBody* b, CollisionInfo& info ) {

    if ( a->collider.type == RIGID_BODY_BOX )
    {
        if ( b->collider.type == RIGID_BODY_BOX )
            return CollideBoxBox( a, b, info );
        else if ( b->collider.type == RIGID_BODY_SPHERE )
            return  CollideCircleBox( b, a, info );
    }

    else if ( a->collider.type == RIGID_BODY_SPHERE )
    {
        if ( b->collider.type == RIGID_BODY_SPHERE )
            return CollideCircleCircle( a, b, info );
        else if ( b->collider.type == RIGID_BODY_BOX )
            return CollideCircleBox( a, b, info );
    }

    // Should not get there
    return false;
}

