#include "hvmc_collisions.h"
#include "hvmc_physics.h"


#define Plop std::cout << "Plop" << std::endl;


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
    continuousDetectionBox(*rb1, *rb2);

    info.rb1 = rb1;
    info.rb2 = rb2;

    double overlapX = 0.0;
    double overlapY = 0.0;

    double rb1XMin = rb1->position.x - rb1->collider.dims.x * 0.5f;
    double rb1XMax = rb1->position.x + rb1->collider.dims.x * 0.5f;
    double rb1YMin = rb1->position.y - rb1->collider.dims.y * 0.5f;
    double rb1YMax = rb1->position.y + rb1->collider.dims.y * 0.5f;

    double rb2XMin = rb2->position.x - rb2->collider.dims.x * 0.5f;
    double rb2XMax = rb2->position.x + rb2->collider.dims.x * 0.5f;
    double rb2YMin = rb2->position.y - rb2->collider.dims.y * 0.5f;
    double rb2YMax = rb2->position.y + rb2->collider.dims.y * 0.5f;

    bool rb1Sup;    // Vrai si rb1 est au dessus de rb2
    bool rb1Gauche; // Vrai si rb1 est à gauche de rb2

    // Test d'un overlap en abscisse (X)
    //
    //    RB2---------------------+
    //    |                       |
    //    |                       |
    //    |        RB1-------+ . . . . . . .
    //    |        |         |    |        .
    //    |        |         |    |        .
    //    +--------|---------|----+        .
    //   Min       |         |   Max       .
    //             |         |             .
    //             +---------+ . . . . . . .
    //            Min       Max           Max
    //
    if((rb1XMin >= rb2XMin) && (rb1XMin <= rb2XMax))
    {
        overlapX = fabs(rb1XMin - rb2XMax);
    }
    //
    //    RB1---------------------+
    //    |                       |
    //    |                       |
    //    |        RB2-------+ . . . . . . .
    //    |        |         |    |        .
    //    |        |         |    |        .
    //    +--------|---------|----+        .
    //   Min       |         |   Max       .
    //             |         |             .
    //             +---------+ . . . . . . .
    //            Min       Max           Max
    //
    else if((rb2XMin >= rb1XMin) && (rb2XMin <= rb1XMax))
    {
        overlapX = fabs(rb2XMin - rb1XMax);
    }
    // Aucune intersection
    else
    {
        return false;
    }


    // Test d'un overlap en ordonnée (Y)
    //
    //    . . . . . . . . . . . . . Max
    //    .                       .
    //    .                       .
    //    .        RB2---------------------+ Max
    //    .        |              .        |
    //    RB1------|--------------+ Max    |
    //    |        |              |        |
    //    +--------|--------------+ Min    |
    //             |                       |
    //             +-----------------------+ Min
    //
    if((rb1YMin >= rb2YMin) && (rb1YMin <= rb2YMax))
    {
        overlapY = fabs(rb1YMin - rb2YMax);
    }
    //
    //    . . . . . . . . . . . . . Max
    //    .                       .
    //    .                       .
    //    .        RB1---------------------+ Max
    //    .        |              .        |
    //    RB2------|--------------+ Max    |
    //    |        |              |        |
    //    +--------|--------------+ Min    |
    //             |                       |
    //             +-----------------------+ Min
    //
    else if((rb2YMin >= rb1YMin) && (rb2YMin <= rb1YMax))
    {
        overlapY = fabs(rb2YMin - rb1YMax);
    }
    // Aucune intersection
    else
    {
        return false;
    }



    //
    // A partir d'ici il y a bien collision
    //

    // On détermine les positions relatives des objets
    rb1Gauche = rb1->position.x < rb2->position.x;
    rb1Sup = rb1->position.y > rb2->position.y;

    // Collision suivant l'axe des abscisses (X)
    if (overlapX < overlapY){
        info.interPenetrationDistance = overlapX;

        // On détermine le sens de la normale
        if(rb1Gauche)
        {
            info.contactNormal = vec2{1.f, 0.f};
        }
        else
        {
            info.contactNormal = vec2{-1.f, 0.f};
        }

        // Cas spécifique : Min2 < Min1 < Max1 < Max2
        if(rb1YMin >= rb2YMin && rb1YMax <= rb2YMax)
        {
            // RB1 à droite :
            //
            // --> Contact normal
            // X   Contact position
            //
            // RB2----------------+
            // |                  |
            // |             RB1------+
            // |             |    |   |
            // |             X <--|   |
            // |             |    |   |
            // |             +--------+
            // |                  |
            // +------------------+
            //
            // RB1 à gauche :
            //
            // Même schéma symétrique (selon un axe vertical)

            // (rb1Gauche ? 1 : -1) <=> Si RB1 est à droite, le point de contact est à gauche de RB1 donc -
            //                          Si RB1 est à gauche, le point de contact est à droite de RB1 donc +
            info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f) * (rb1Gauche ? 1 : -1);
            info.contactPosition.y = rb1->position.y;
            return true;
        }

        // Cas spécifique : Min1 < Min2 < Max2 < Max1
        if(rb2YMin >= rb1YMin && rb2YMax <= rb1YMax)
        {
            // RB1 à gauche :
            //
            // --> Contact normal
            // X   Contact position
            //
            // RB1----------------+
            // |                  |
            // |             RB2------+
            // |             |    |   |
            // |         --> |    X   |
            // |             |    |   |
            // |             +--------+
            // |                  |
            // +------------------+
            //
            // RB1 à droite :
            //
            // Même schéma symétrique (selon un axe vertical)

            // (rb1Gauche ? 1 : -1) <=> Si RB1 est à droite, le point de contact est à gauche de RB1 donc -
            //                          Si RB1 est à gauche, le point de contact est à droite de RB1 donc +
            info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f) * (rb1Gauche ? 1 : -1);
            info.contactPosition.y = rb2->position.y;
            return true;
        }


        // Calcul du point de contact
        if (rb1Sup)
        {
            if (rb1Gauche)
            {
                info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f);
                info.contactPosition.y = rb1->position.y - (rb1->collider.dims.y * 0.5f) + overlapY * 0.5f;
            }
            else
            {
                info.contactPosition.x = rb1->position.x - (rb1->collider.dims.x * 0.5f);
                info.contactPosition.y = rb1->position.y - (rb1->collider.dims.y * 0.5f) + overlapY * 0.5f;
            }
        }
        else{
            if (rb1Gauche){
                info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f);
                info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f) - overlapY * 0.5f ;
            }
            else{
                info.contactPosition.x = rb1->position.x - (rb1->collider.dims.x * 0.5f);
                info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f) - overlapY * 0.5f ;
            }
        }
    }
    // Collision suivant l'axe des ordonnées (Y)
    else
    {
        info.interPenetrationDistance = overlapY;

        // On détermine le sens de la normale
        if(rb1Sup)
        {
            info.contactNormal = vec2{0.f, -1.f};
        }
        else
        {
            info.contactNormal = vec2{0.f, 1.f};
        }

        // Cas spécifique : Min2 < Min1 < Max1 < Max2
        if(rb1XMin >= rb2XMin && rb1XMax <= rb2XMax)
        {
            // RB1 en haut :
            //
            // --> Contact normal
            // X   Contact position
            //
            // RB1-----------------+
            // |                   |
            // |                   |
            // |                   |
            // |                   |
            // |         |         |
            // |         v         |
            // |    RB2-------+    |
            // |    |         |    |
            // +----|----X----|----+
            //      |         |
            //      |         |
            //      +---------+
            //
            // RB1 en bas :
            //
            // Même schéma symétrique (selon un axe horizontal)

            // (rb1Sup ? -1 : 1) <=> Si RB1 est au dessus,  le point de contact est au dessous de RB1 donc -
            //                       Si RB1 est au dessous, le point de contact est au dessus  de RB1 donc +
            info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f) * (rb1Sup ? -1 : 1);
            info.contactPosition.x = rb1->position.x;
            return true;
        }

        // Cas spécifique : Min1 < Min2 < Max2 < Max1
        if(rb2XMin >= rb1XMin && rb2XMax <= rb1XMax)
        {
            // RB1 en bas :
            //
            // --> Contact normal
            // X   Contact position
            //
            // RB2-----------------+
            // |                   |
            // |                   |
            // |                   |
            // |                   |
            // |         |         |
            // |         v         |
            // |    RB1-------+    |
            // |    |         |    |
            // +----|----X----|----+
            //      |         |
            //      |         |
            //      +---------+
            //
            // RB1 en haut :
            //
            // Même schéma symétrique (selon un axe horizontal)

            // (rb1Sup ? -1 : 1) <=> Si RB1 est au dessus,  le point de contact est au dessous de RB1 donc -
            //                       Si RB1 est au dessous, le point de contact est au dessus  de RB1 donc +
            info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f) * (rb1Sup ? -1 : 1);
            info.contactPosition.x = rb2->position.x;
            return true;
        }

        // Calcul du point de contact
        if(rb1Sup)
        {
            if(rb1Gauche)
            {
                info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f) - overlapX * 0.5f ;
                info.contactPosition.y = rb1->position.y - (rb1->collider.dims.y * 0.5f);
            }
            else
            {
                info.contactPosition.x = rb1->position.x - (rb1->collider.dims.x * 0.5f) + overlapX * 0.5f ;
                info.contactPosition.y = rb1->position.y - (rb1->collider.dims.y * 0.5f);
            }
        }
        else
        {
            if(rb1Gauche)
            {
                info.contactPosition.x = rb1->position.x + (rb1->collider.dims.x * 0.5f) - overlapX * 0.5f ;
                info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f);
            }
            else
            {
                info.contactPosition.x = rb1->position.x - (rb1->collider.dims.x * 0.5f) + overlapX * 0.5f ;
                info.contactPosition.y = rb1->position.y + (rb1->collider.dims.y * 0.5f);
            }
//            if(info.debug) std::cout << info.interPenetrationDistance << info.contactPosition << info.contactNormal << std::endl;
        }
    }

    return true;
}



f32 Clamp(f32 v, f32 min, f32 max)
{
    if(v < min) return min;
    if(v > max) return max;
    return v;
}

std::ostream& operator<< (std::ostream& os, vec2 v)
{
    return os << "(" << v.x << ", " << v.y << ")";
}

bool continuousDetectionBox(RigidBody& box, RigidBody& body)
{
    Ray ray(body.previousPosition, body.position - body.previousPosition);

    // L'objet ne bouge pas
    if(ray.dir.x == 0 && ray.dir.y == 0)
    {
        return false;
    }

    // L'objet est bien en mouvement
    f32 x_min = box.position.x - box.collider.dims.x * 0.5;
    f32 x_max = box.position.x + box.collider.dims.x * 0.5;
    f32 y_min = box.position.y - box.collider.dims.y * 0.5;
    f32 y_max = box.position.y + box.collider.dims.y * 0.5;

    f32 t_min = -INFINITY;
    f32 t_max =  INFINITY;
    if(ray.dir.x) // L'objet a une composante de déplacement en X
    {
        f32 t_x_min = (x_min - ray.pos.x) * ray.invDir.x;
        f32 t_x_max = (x_max - ray.pos.x) * ray.invDir.x;
        t_min = std::min(t_x_min, t_x_max);
        t_max = std::max(t_x_min, t_x_max);
    }
    if(ray.dir.y) // L'objet a une composante de déplacement en Y
    {
        f32 t_y_min = (y_min - ray.pos.y) * ray.invDir.y;
        f32 t_y_max = (y_max - ray.pos.y) * ray.invDir.y;
        t_min = std::max(t_min, std::min(t_y_min, t_y_max));
        t_max = std::min(t_max, std::max(t_y_min, t_y_max));
    }

    // Si collision détectée
    if((t_max > t_min) && (t_min > 0) && (norm(t_min * ray.dir) <= norm(ray.dir)))
    {
//        std::cout << x_min << ", " << x_max << " : " << norm(t_min * ray.dir) << ", " << norm(ray.dir);
//        std::cout << body.previousPosition << body.position << body.previousPosition + ray.dir * t_min << std::endl;
        body.position = body.previousPosition + ray.dir * t_min;
//        body.SetKinematic();
        return true;
    }
    return false;
}

bool CollideBoxCircle( RigidBody* box, RigidBody* cir, CollisionInfo &info){

    // Replace l'objet sur la surface de l'objet détecté
    continuousDetectionBox(*box, *cir);

    // Calcule l'intersection classique
    vec2 center_box = box->position;
    vec2 center_cir = cir->position;
    f32 radius_cir = cir->collider.radius;
    f32 hbw = box->collider.dims.x * 0.5; // Half Box Width
    f32 hbh = box->collider.dims.y * 0.5; // Half Box Height

    vec2 v_box_cir = center_cir - center_box; // Vector Box->Circle
    vec2 v_clamp_box_cir = vec2{Clamp(v_box_cir.x, -hbw, hbw), Clamp(v_box_cir.y, -hbh, hbh)};

    vec2 contact = (center_box + v_clamp_box_cir);

    vec2 v_cir_contact = contact - center_cir; // Vector Circle->Contact

    // Si intersection détectée
    if(norm(v_cir_contact) <= radius_cir)
    {
        info.rb1 = box;
        info.rb2 = cir;
        info.contactPosition = contact;

        f32 x_min_box = box->position.x - box->collider.dims.x * 0.5f;
        f32 x_max_box = box->position.x + box->collider.dims.x * 0.5f;
        f32 y_min_box = box->position.y - box->collider.dims.y * 0.5f;
        f32 y_max_box = box->position.y + box->collider.dims.y * 0.5f;
        // Collision suivant l'axe des abscisses (X)
        if(contact.y > y_min_box && contact.y < y_max_box)
        {
            // (v_box_cir.x < 0) <=> Circle à gauche de Box, donc normale vers la gauche
            //             Sinon <=> Circle à droite de Box, donc normale vers la droite
            float c = ((v_box_cir.x < 0) ? -1.f : 1.f);
            info.contactNormal = vec2{c, 0.f};
        }
        // Collision suivant l'axe des ordonnées (Y)
        else if(contact.x > x_min_box && contact.x < x_max_box)
        {
            // (v_box_cir.y > 0) <=> Circle au dessus  de Box, donc normale vers le haut
            //             Sinon <=> Circle au dessous de Box, donc normale vers le bas
            float c = ((v_box_cir.y > 0) ? 1.f : -1.f);
            info.contactNormal = vec2{0.f, c};
        }
        // Collision sur l'angle (TODO : A retravailler)
        else
        {
            info.contactNormal = Normalize(v_box_cir);
        }
        info.interPenetrationDistance = radius_cir - norm(contact - center_cir);
        return true;
    }
    return false;
}

void CollisionInfo::Solve() const
{

    // Renommage des données
    f32 im_a = rb1->im;
    f32 im_b = rb2->im;
    f32 iI_a = rb1->iI;
    f32 iI_b = rb2->iI;
    vec2 v_a = rb1->velocity;
    vec2 v_b = rb2->velocity;
    f32 w_a = rb1->angularVelocity;
    f32 w_b = rb2->angularVelocity;

    vec2 n = contactNormal; //

    vec2 r_a =  contactPosition - rb1->position;
    vec2 r_b =  contactPosition - rb2->position;
    vec2 v_rel = (v_b + Cross(w_b, r_b)) - (v_a + Cross(w_a, r_a)); //

//    f32 e = -(m_a * a_a - m_b * a_b) / (m_a * v_a - m_b * v_b);
    f32 e = 0.8f;

    f32 J = (-(1 + e) * Dot(v_rel, n)) / (im_a + im_b + iI_a * Cross(r_a, n) + iI_b * Cross(r_b, n)); //

    vec2 j_a = -J * n;
    vec2 j_b =  J * n;

    if (Dot(v_rel,n) < 0){
        vec2 p = vec2{0.f, 0.f};
        rb1->ApplyImpulse(j_a, p);
        rb2->ApplyImpulse(j_b, p);
    }
}

void CollisionInfo::PositionCorrection() const
{
    static const f32 threshold = 0.01f;
    static const f32 percentage = 0.5f;
    f32 im = rb1->im + rb2->im;
    if(im == 0) return;
    vec2 correction = (std::max(interPenetrationDistance - threshold, 0.0) / im) * percentage * contactNormal;
    rb1->position = rb1->position - rb1->im * correction;
    rb2->position = rb2->position + rb2->im * correction;
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
            return CollideBoxCircle( a, b, info );
    }

    else if ( a->collider.type == RIGID_BODY_SPHERE )
    {
        if ( b->collider.type == RIGID_BODY_SPHERE )
            return CollideCircleCircle( a, b, info );
        else if ( b->collider.type == RIGID_BODY_BOX )
            return CollideBoxCircle( b, a, info );
    }

    // Should not get there
    return false;
}

