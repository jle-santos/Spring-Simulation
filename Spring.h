#pragma once
#include <cmath>

#define in_to_m 0.0254
#define Pi 3.14159

class CSpring
   {
   public:
      float _k;         //Spring constant
      float _Fc;        //Force at rest
      float _T;         //Tension force (extended)

      float _Lo;        //Length at rest
      float _Ls;        //Extended length
      float _angle;     //Angle from X-Axis
      float _lambda_s;  //

      float _i;         //X Pin location
      float _j;         //Y Pin location

      //Calculate length, force, lambda, and angle
      void calculate_parameters(float xc, float yc);

      CSpring();
      ~CSpring();
   };

