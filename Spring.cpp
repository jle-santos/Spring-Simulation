#include "stdafx.h"
#include "Spring.h"


CSpring::CSpring()
   {
   }

void CSpring::calculate_parameters(float xc, float yc)
   {
   _Ls = sqrt((pow((xc - _i), 2) + (pow((yc - _j),2))));

   _T = _Fc + _k*(_Ls - _Lo);

   _lambda_s = _Fc - _k*_Lo;

   _angle = atan2(((xc - _i) / _Ls), (yc - _j) / _Ls);

   //Compensate for angles less than 1
   if (_angle < 0)
      {
      _angle = 2*Pi + _angle;
      }
   }

CSpring::~CSpring()
   {
   }
