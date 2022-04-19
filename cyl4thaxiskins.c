/*****************************************************************
* Description: cyl4thaxiskins.c
*   Kinematics for a cartesian machine with a rotatory axis A
*   parallel to the x axis.
*
*   m_* coordinates of machine (all coordinates here are in the absolute g-code coordinate system)
*   to_* coordinates for the tool offset cannot be used
*   u,v,w used for other purposes
*
*   pos.tran.x|u|z coordinates of the tool tip
*   pos.a, or a is the rotation of the tool's axis around the x-axis.
*               The value is zero if the z-axis and the tool's axis line up.
*   pos.tran.y is a virtual axis along the circumference of a cylinder with
*   pos.w = cylinder radius, and with
*   pos.v = offset adjusted to have y=0 at the vertical rotation center plane

*   h1 is the height of the rotatory axis, measured from the machines origin in z-direction.
*   Y <--> U are exchanged to use orginial xu plane as xy plane required for
*   arc G2,G3 and G6 spline moves 

* Author: Matthias Walser
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2022 All rights reserved.
*
* Last change: 01/2022
*******************************************************************
*/

#include "kinematics.h"   /* decls for kinematicsForward, etc. */
#include "rtapi.h"		    /* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_math.h"
#include "hal.h"

struct haldata {
    hal_float_t *h1;
} *haldata = 0;

#define H1 (*(haldata->h1))

#define d2r(d) ((d)*PM_PI/180.0)
#define r2d(r) ((r)*180.0/PM_PI)

void toolOffset(double *to_x, double *to_y, double *to_z, const double sin_a, const double cos_a, const double u, const double v, const double w)
{
      // cannot use u,v,w for tool offset, used for other purposes
	*to_x = 0*u; //use G92 X[offset] to compensate
	*to_y = v;   //used to position tool on xz plane
	*to_z = 0*w; //use G92 Z[offset] to compensate
}

int kinematicsForward(const double* joint,
                      EmcPose* pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
  // assumes y & v set such that rotation axis is at y=0
  // move on cylindrical surface with cyl-radius: = radius
  double a = d2r(joint[3]); //angle in radiant
  double radius=joint[8];  //cylinder radius (a in rad to distance in mm)

  double sin_a = sin(a);   double cos_a = cos(a);
  double u;                double m_x, m_y, m_z;
  double to_x, to_y, to_z;
  toolOffset(&to_x, &to_y, &to_z, sin_a, cos_a, joint[6], joint[7], joint[8]);
 
  radius=joint[8];     //radius (a in rad to distance in mm)
 
  //calculate coordinates of machine  
  m_x = joint[0]; // physical x axis
  m_y = joint[6]; // physical y axis
    u = a*radius; // CAD y position (virtual axis taking user commands)
  m_z = joint[2]; // physical y axis

  // calculate coordinates of commanded point
  // from machine pos. and tool_offsets
  pos->tran.x = m_x - to_x; // here to_x=0
  pos->u      = m_y - to_y; // offset position of xz plane
  pos->tran.z = m_z - to_z; // here to_z=0
  pos->a = joint[3];        // physical rotation about x-axis
  pos->b = joint[4];        // not used
  pos->c = joint[5];        // not used
  pos->tran.y = u; //store virtual y position to grap in inverse kin
  pos->v = joint[7]; //store physical y-offset of xz plane
  pos->w = joint[8]; //store conversion radius to grap in inverse kin

  return 0;
}

int kinematicsInverse(const EmcPose* pos,
                      double* joint,
                      const KINEMATICS_INVERSE_FLAGS* iflags,
                      KINEMATICS_FORWARD_FLAGS* fflags)
{
  double a = d2r(pos->a);   // a from degree to angle in rad
  double radius=(pos->w);   // cylinder radius in mm
  // JOINT1: (pos->tran.y); // CAD y position in mm (virtual axis)
  // JOINT6: (pos->u);      // machine y position, real  axis parked
  // Y <--> U are exchanged to use 'orginial xu plane' as xy plane

  double sin_a = sin(a);      double cos_a = cos(a);
  double m_x, m_y, m_z, m_a;  double to_x, to_y, to_z;
  toolOffset(&to_x, &to_y, &to_z, sin_a, cos_a, pos->u, pos->v, pos->w);
  
  // calculate coordinates of machine
  // from tool tip actual pos. and tool_offsets
  m_x = (pos->tran.x) + to_x;  //use G92 X[offset] to compensate
  m_y = (pos->u)      + to_y;  //to intially position tool on xz plane
  m_z = (pos->tran.z) + to_z;  //use G92 Z[offset] to compensate

  if(-1e-6 < radius && radius < 1e-6) // if radius is close to zero
  {                                   // avoid singularity and discont.
    m_a = (pos->a); //for too small radius, recover trivkins XYZA
   // Issue radius=0 only at y=0 in teleoperation (or if joint3=0)
   //  Send first G0 Y0, then G1 W0 (radius=to 0), back to XYZA
   // Issue radius>0 only at y=0 in teleoperation (or if joint3=0)
   //  Send G0 Y0 (actual move is in a), then G92 A0
   //  then G0 W10 (radius=10), updated is a=r2d(y/radius) in degree
  }
  else
  {
    m_a = r2d(pos->tran.y)/radius; // yaxis to angle in rad
  }
	
  //caluclate joint values
  joint[0] = m_x;
  joint[6] = m_y;
  joint[2] = m_z;
  joint[3] = m_a; //Commands on A:pos->a will have no effect

  joint[4] = pos->b;       // not used
  joint[5] = pos->c;       // not used
  joint[1] = pos->tran.y;  // Commands on Y will rotate axis A
  joint[7] = pos->v;       // physical yposition of xz plane
  joint[8] = pos->w;       // radius of cylinder to machine

  return 0;
}

int kinematicsHome(EmcPose * world,
                   double * joint,
                   KINEMATICS_FORWARD_FLAGS * fflags,
                   KINEMATICS_INVERSE_FLAGS * iflags)
{
  /* use joint, set world */
  return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
  return KINEMATICS_BOTH;
}


EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);

int comp_id;

int rtapi_app_main(void) {
  int res=0;
  
  comp_id = hal_init("cyl4thaxiskins");
  if (comp_id < 0) return comp_id;
    
  haldata = hal_malloc(sizeof(struct haldata));
  if (!haldata) goto error;

  if((res = hal_pin_float_new("cyl4thaxiskins.H1", HAL_IN, &(haldata->h1), comp_id)) < 0) goto error;   
  H1 = 0.;
  
  hal_ready(comp_id);
  return 0;
  
error:
  hal_exit(comp_id);
  return res;
}

void rtapi_app_exit(void) {
  hal_exit(comp_id);
}

