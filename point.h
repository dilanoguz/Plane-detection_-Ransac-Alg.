#pragma once
using namespace std;

class Point
{
public:
	double x, y, z, val;
	//constructor
	Point(double xx, double yy, double zz, double value);
	Point() {};
};

Point::Point(double xx, double yy, double zz, double value){
		x = xx;
		y = yy;
		z= zz;
		val = value;
	}