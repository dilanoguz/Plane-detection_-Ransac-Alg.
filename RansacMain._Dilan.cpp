#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <iterator>
#include <cstdio>
#include <cmath>

#include "Point.h"

using namespace std;

// ====== Develop of Functions ==========
void Ransac_alg(vector<Point> dataset) {

	clock_t start = clock();
	//====== Parameters of Ransac Algorithm ============
	// If the remaining points are less than 8% of the whole dataset, then STOP searching for a new plane
	int min_perc_pnt_check = static_cast<int>(dataset.size()) *.08;   
	// Threshold between point and plane
	double threshold_plane_pnt = 0.02;           
	// % of Points to create a plane
	int minimum_pnt = static_cast<int>(dataset.size()) *.05;  

	vector<Point> points_in_plane;
	int plane_number = 0;
	int count_loops = 0; //Count the iteration in do-while loop
	int pnt_in_plane;
	double Random3_Points[3][3];
	double (plane)[4];
	
	do {
		bool condition = false;
		random_three_pnt(dataset, Random3_Points, threshold_plane_pnt);
		plane_parameters(Random3_Points, plane);
		pnt_in_plane = 0;

		double plane_norm = sqrt(pow(plane[0], 2) + pow(plane[1], 2) + pow(plane[2], 2));
		double distance_pnt_plane;

		for (int i = 0; i < dataset.size(); i++) {
			dataset[i].val = 0;
			//distance of a point and the plane
			distance_pnt_plane = abs((plane[0] * dataset[i].x + plane[1] * dataset[i].y + plane[2] * dataset[i].z + plane[3])) / plane_norm;
			if (distance_pnt_plane <=  threshold_plane_pnt) {
				pnt_in_plane++;
				dataset[i].val = 1;
			}
		}
		if ((pnt_in_plane - minimum_pnt) > 0){ condition = true; }
			
		if (condition == true) {
			count_loops = 0;
			plane_number++;
			//Create vector with points that belong in the current Plane
			for (auto pnt : dataset) {
				if (pnt.val == 1) {
					points_in_plane.push_back(pnt);
				}
			}

			//Save the points of the specific plane
			string name = ("./plane_" + to_string(plane_number) + ".txt");
			
			cout << " The dataset of new plane have been saved and have size of:  " << points_in_plane.size() << endl;
			save_data(name, points_in_plane);

			//Erase from the initial dataSet the points that belongs to the plane
			for (vector<Point>::iterator it = dataset.end()-1; it != dataset.begin() ; it--) {
				if (it->val == 1){
					it = dataset.erase(it);
				}
			}

			//Erase points that belong to the vector of points_in_plane
			points_in_plane.clear();

			cout << "The Remaining points after the plane creations are: \t" << dataset.size() << endl;
			cout << endl;
		}
		count_loops++;
	} while (dataset.size()>min_perc_pnt_check && count_loops<=250000);
		
	cout << "Calculation of time:  " << start - clock_t() << endl;

	save_data("./RemainPoints.txt", dataset);
	cout << endl << "Save of remain points : " << dataset.size();

	if (count_loops > 250000) {
		cout << endl << "failed to detect another plane" << endl << "press enter to continue";
		cin.get();
	}
}
void plane_parameters(double(&point_coord)[3][3], double(&plane)[4]) {
	plane[0] = (point_coord[1][1] - point_coord[0][1])*(point_coord[2][2] - point_coord[0][2])
		- (point_coord[2][1] - point_coord[0][1])*(point_coord[1][2] - point_coord[0][2]);
	plane[1] = (point_coord[1][2] - point_coord[0][2])*(point_coord[2][0] - point_coord[0][0])
		- (point_coord[2][2] - point_coord[0][2])*(point_coord[1][0] - point_coord[0][0]);
	plane[2] = (point_coord[1][0] - point_coord[0][0])*(point_coord[2][1] - point_coord[0][1])
		- (point_coord[2][0] - point_coord[0][0])*(point_coord[1][1] - point_coord[0][1]);
	plane[3] = -(plane[0] * point_coord[0][0] + plane[1] * point_coord[0][1] + plane[2] * point_coord[0][2]);
	return;
}
void random_three_pnt(vector<Point> data, double (&ThreePoint)[3][3], double thres) {
	int  p1, p2, p3;
	double dist1, dist2, dist3;
	do {
		p1 = rand() % data.size();
		p2 = rand() % data.size();
		p3 = rand() % data.size();

		dist1 = int(sqrt(pow((data[p1].x - data[p2].x), 2) + pow((data[p1].y - data[p2].y), 2)
			+ pow((data[p1].z - data[p2].z), 2)));
		dist2 = int(sqrt(pow((data[p1].x - data[p3].x), 2) + pow((data[p1].y - data[p3].y), 2)
			+ pow((data[p1].z - data[p3].z), 2)));
		dist3 = int(sqrt(pow((data[p2].x - data[p3].x), 2) + pow((data[p2].y - data[p3].y), 2)
			+ pow((data[p2].z - data[p3].z), 2)));
	}while ((p1 == p2 || p1 == p3 || p2 == p3)||(dist1<=5*thres || dist2<= 5* thres || dist3 <= 5* thres));


	ThreePoint[0][0] = (data[p1].x);
	ThreePoint[0][1] = (data[p1].y);
	ThreePoint[0][2] = (data[p1].z);
	ThreePoint[1][0] = (data[p2].x);
	ThreePoint[1][1] = (data[p2].y);
	ThreePoint[1][2] = (data[p2].z);
	ThreePoint[2][0] = (data[p3].x);
	ThreePoint[2][1] = (data[p3].y);
	ThreePoint[2][2] = (data[p3].z);
	
	return;
}

vector<Point> read_data(string filename) {
	double x, y, z, value;
	vector<Point> pts;
	ifstream filein;
	filein.open(filename.c_str());
	point pt;
	string line;
	while (getline(filein, line)) 
	{
		filein >> x >> y >> z >> value;
		pt.x = x;
		pt.y = y;
		pt.z = z;
		pt.val = value;
		pts.push_back(pt);
	}
	filein.close();
	return pts;
}
void save_data(string filename, vector<Point> pts) {
	ofstream fileout;
	string red, green, blue;
	red =RandomColor();
	green = RandomColor();
	blue = RandomColor();
	fileout.open(filename.c_str());
	for (int i = 0; i < pts.size(); i++) {
		fileout << pts[i].x << " " << pts[i].y << " " << pts[i].z << " " << pts[i].val<< " " << red << " " << green << " " << blue << endl;
	}
	fileout.close();
}

string RandomColor() {
	int  rc;
	rc = 0+ rand() % (0-250);
		return to_string(rc);
}

// ======= main() ===========
int main() {
	srand(time(NULL));//different random number in each running
	vector<Point> points;
	vector<Point> dataset;

	points = read_data("./Charite-large.ptx");
	cout << " Number of points of the whole dataset: " << points.size() << "\n" << endl;

	//Data with valuable information
	for (auto pnt: points) {
		if (pnt.x == .0 && pnt.y == .0 && pnt.z == .0) continue;
		dataset.push_back(pnt);
	}
	
	cout << "Number of points with information: " << dataset.size() << endl;
	cout << endl;

	//Call the function of Ransac
	Ransac_alg(dataset);

	cout << "\n Success termination of the Ransac Algorithm!!! \t " <<endl;

	system("PAUSE");
	return 0;
}
