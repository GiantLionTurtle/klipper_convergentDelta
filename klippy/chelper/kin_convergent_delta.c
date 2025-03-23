#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord
#include <stdio.h>

struct convdelta_stepper {
    struct stepper_kinematics sk;
    double arm2, length2, length;
    struct coord start;
    struct coord end;
};

double 
dot(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return x1*x2 + y1*y2 + z1*z2;
}
double length2(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double dx = x2-x1;
    double dy = y2-y1;
    double dz = z2-z1;
    return dx*dx+dy*dy+dz*dz;
}

static void 
closest_point(struct convdelta_stepper *ds, struct coord const target, struct coord* closest_pos, double* pos_on_actuator)
{
    struct coord dir;
    dir.x = ds->end.x-ds->start.x;
    dir.y = ds->end.y-ds->start.y;
    dir.z = ds->end.z-ds->start.z;

    struct coord pos_to_start;
    pos_to_start.x = target.x-ds->start.x;
    pos_to_start.y = target.y-ds->start.y;
    pos_to_start.z = target.z-ds->start.z;

    // lerp position [0, 1]
    double lerp = dot(dir.x, dir.y, dir.z, pos_to_start.x, pos_to_start.y, pos_to_start.z) / (ds->length2);

    // find the position in 3d
    closest_pos->x = ds->start.x + dir.x * lerp;
    closest_pos->y = ds->start.y + dir.y * lerp;
    closest_pos->z = ds->start.z + dir.z * lerp;

    // position in 1d along the axis
    *pos_on_actuator = lerp * ds->length;
}
static double
convdelta_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct convdelta_stepper *ds = container_of(sk, struct convdelta_stepper, sk);
    struct coord target = move_get_coord(m, move_time);

    // closest lerp
    double pos_on_actuator = 0.0;
    struct coord closest_pos;
    closest_point(ds, target, &closest_pos, &pos_on_actuator);
    // printf("start={%f, %f, %f}", ds->start.x, ds->start.y, ds->start.z);
    // printf("end={%f, %f, %f}", ds->end.x, ds->end.y, ds->end.z);
    // printf("target={%f, %f, %f}", target.x, target.y, target.z);
    // printf("closest_pos={%f, %f, %f}", closest_pos.x, closest_pos.y, closest_pos.z);

    double dist_to_act2 = dot(closest_pos.x, closest_pos.y, closest_pos.z, target.x, target.y, target.z);
    // printf("dist_to_act2=%f", dist_to_act2);
    // printf("pos_on_actuator=%f", pos_on_actuator);
    if(dist_to_act2 >= ds->arm2) { // impossible or limit position
        return pos_on_actuator >= ds->length ? ds->length : 0.0;
    }

    // // Not directly perpendicular, not too far
    // // solve for second cathede of a right triangle
    double offset_along_actuator = sqrt(ds->arm2 - dist_to_act2);
    
    // printf("offset_along_actuator=%f", offset_along_actuator);
    // printf("dslength=%f", ds->length);
    // double plus_option = pos_on_actuator + offset_along_actuator;
    double minus_option = pos_on_actuator - offset_along_actuator;

    // if(plus_option >= 0.0 && plus_option <= ds->length) {
        // printf("gotoplus=%f", plus_option);
    //     return plus_option;
    // } else if(minus_option >= 0.0 && minus_option <= ds->length) {
    //     printf("gotominus=%f", minus_option);
    //     return minus_option;
    // }
    // printf("gotoplus%f=", plus_option);
    // if(minus_option < 0.0) {
    //     // printf("")
    //     return 0.0;
    // }
    // printf("gotominus=%f\n", minus_option);
    // if((int)ds->start.x == 206) {
    //     // printf("start={%f, %f, %f}", ds->start.x, ds->start.y, ds->start.z);
    //     // printf("end={%f, %f, %f}", ds->end.x, ds->end.y, ds->end.z);
    //     printf("target={%f, %f, %f}", target.x, target.y, target.z);
    //     printf("closest_pos={%f, %f, %f}", closest_pos.x, closest_pos.y, closest_pos.z);
    //     printf("dist_to_act2=%f", dist_to_act2);
    //     printf("pos_on_actuator=%f", pos_on_actuator);
    //     // printf("dslength=%f", ds->length);
    //     printf("gotominus=%f\n", minus_option);
    // }
    return minus_option;
}

struct stepper_kinematics * __visible
convergent_delta_stepper_alloc( double arm2, double x1, double y1, double z1, double x2, double y2, double z2)
{
    struct convdelta_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));

    ds->arm2 = arm2;
    ds->length2 = length2(x1, y1, z1, x2, y2, z2);
    ds->length = sqrt(ds->length2);

    ds->start.x = x1;
    ds->start.y = y1;
    ds->start.z = z1;
    
    ds->end.x = x2;
    ds->end.y = y2;
    ds->end.z = z2;

    ds->sk.calc_position_cb = convdelta_stepper_calc_position;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &ds->sk;
}
