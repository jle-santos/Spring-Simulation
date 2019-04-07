// Spring_Sim.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Spring.h"
#include <vector>
#include <fstream>
#include <iostream>

//Total Springs in the system 
#define TOTAL_SPRINGS 3

//Spring names, RED = 0, GREEN = 1, BLUE = 2
enum{RED, GREEN, BLUE};

void load_constants();

//Equations used in calculations 
double Px(CSpring spr);          //Phi x
double Py(CSpring spr);          //Phy y

//Partial Derivative Functions, accepts a spring object
double dPxdx(CSpring spr);       //Partial Phi x, Partial X
double dPxdy(CSpring spr);       //Partial Phi x, Partial Y
double dPydx(CSpring spr);       //Partial Phi y, Partial X
double dPydy(CSpring spr);       //Partial Phi y, Partial Y

//Newton's Method Derivation, accepts the guess/iteration guess
void newton_method(double x, double y);

std::vector<CSpring> Spring;

double Xcm1, Ycm1, Xc0, Yc0, Xcm, Ycm;
double l_error = 1;

int main()
{
    //Load constants from text file
    std::ifstream spring_constants("SpringConstants.txt");

    if (spring_constants.is_open())
       {
       CSpring spring_data;
       while (spring_constants >> spring_data._k >> spring_data._Fc >> spring_data._Lo >> spring_data._i >> spring_data._j)
          {
          Spring.push_back(spring_data);
          }
       spring_constants.close();
       }

    std::cout << "Loaded data from SpringConstants.txt\n";

    //Ask for guesses in inches
    std::cout << "Guess Xc: ";
    std::cin >> Xc0;
    std::cout << "Guess Yc: ";
    std::cin >> Yc0;

    //Convert guesses to meters
    Xc0 = Xc0 * in_to_m;
    Yc0 = Yc0 * in_to_m;

    //Convert spring indexes to meters

    for (int i = 0; i < TOTAL_SPRINGS; i++)
       {
       Spring[i]._i = Spring[i]._i * in_to_m;
       Spring[i]._j = Spring[i]._j * in_to_m;
       }

    bool exit = false;

    int z = 0;

    while (exit == false)
       {

       newton_method(Xc0, Yc0);

       if (z == 10)
          {
          exit = true;
          }
       else
          {
          Xc0 = Xcm1;
          Yc0 = Ycm1;
          std::cout << Xcm1/in_to_m << " " << Ycm1/in_to_m << " " << l_error <<"\n";
          }
       z++;
       }

    while (true) {}
    return 0;
}
	void newton_method(double x, double y)
	{
		int iterate = 1;
		
      //Updates forces, lambda, and coordinates based on guess/iterations 

      Spring[RED].calculate_parameters(x, y);
      Spring[GREEN].calculate_parameters(x, y);
      Spring[BLUE].calculate_parameters(x, y);

		//Summation of all the partial derivates of each spring
		
		double sum_Px = Px(Spring[RED]) + Px(Spring[GREEN]) + Px(Spring[BLUE]);                //Sum of Phi x
		double sum_Py = Py(Spring[RED]) + Py(Spring[GREEN]) + Py(Spring[BLUE]);                //Sum of Phi y
		
		double sum_dPxdx = dPxdx(Spring[RED]) + dPxdx(Spring[GREEN]) + dPxdx(Spring[BLUE]);    //Sum of Partial Phi x, Partial X
		double sum_dPxdy = dPxdy(Spring[RED]) + dPxdy(Spring[GREEN]) + dPxdy(Spring[BLUE]);    //Sum of Partial Phi x, Partial Y
		double sum_dPydx = dPydx(Spring[RED]) + dPydx(Spring[GREEN]) + dPydx(Spring[BLUE]);    //Sum of Partial Phi y, Partial X
		double sum_dPydy = dPydy(Spring[RED]) + dPydy(Spring[GREEN]) + dPydy(Spring[BLUE]);    //Sum of Partial Phi y, Partial Y
		
      //Iteration equations
      Xcm = ((sum_Px*sum_dPydy) - (sum_Py*sum_dPxdy)) / ((sum_dPxdx*sum_dPydy) - (sum_dPydx*sum_dPxdy));    //Develops new Xcm point
      Ycm = ((-sum_Px*sum_dPydx) + (sum_Py*sum_dPxdx)) / ((sum_dPxdx*sum_dPydy) - (sum_dPydx*sum_dPxdy));   //Develops new Ycm point

		Xcm1 = x - Xcm;      //Add guess (X) and Xcm to develop new point
		Ycm1 = y - Ycm;      //Add guess (Y) and Ycm to develop new point
	   
      l_error = sqrt((Xcm1*Xcm1) + (Ycm1*Ycm1));
      }


   double Px(CSpring spr)
   {
	   double px;
	   px = spr._T*cos(spr._angle);
	   return px;
   }
   
   double Py(CSpring spr)
   {
		double py;
		py = spr._T*sin(spr._angle);
		return py;
   }
   

   double dPxdx(CSpring spr)
   {
	   double dpxdx;
	   dpxdx = (spr._lambda_s*(pow(cos(spr._angle), 2) - spr._T)) / spr._Ls;
	   return dpxdx;
   }
   
   
   double dPxdy(CSpring spr)
   { 
		double dpxdy;
		dpxdy = spr._lambda_s*cos(spr._angle)*sin(spr._angle) / spr._Ls;
		return dpxdy;
   }
   
   
   double dPydx(CSpring spr)
   {
	   double dpydx;
	   dpydx = spr._lambda_s*cos(spr._angle)*sin(spr._angle) / spr._Ls;
	   return dpydx;
   }
   
   double dPydy(CSpring spr) 
   {
	   double dpydy;
	   dpydy = (spr._lambda_s*(pow(sin(spr._angle), 2) - spr._T)) / spr._Ls;
	   return dpydy;
   }
