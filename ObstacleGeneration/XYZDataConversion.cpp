#include <iostream>
#include <cstdio>
#include <fstream>
#include <string>


using namespace std;
int main()
{
	bool Point_OBJ = 0;
	if (Point_OBJ) 
	{
		cout.precision(10);
		string line;
		ifstream myfile("C:\\Users\\ccc0032\\Documents\\HeightMap.xyz");
		ofstream OutFile;
		OutFile.open("C:\\Users\\ccc0032\\Documents\\Refined.pnt", ios::out | ios::binary);
		double FList[3] = {};
		int VertexCount = 0;
		if (myfile.is_open())
		{

			while (getline(myfile, line))
			{
				VertexCount++;
				FList[0] = stod(line.substr(0, 10));
				line = line.substr(11);
				FList[1] = stod(line.substr(0, 11));
				line = line.substr(12);
				FList[2] = stod(line.substr(0, 6));
				cout << VertexCount << "\n";
				OutFile.write((char*)&FList, sizeof(double) * 3);

			}
			myfile.close();
			OutFile.close();
		}

		ifstream ReadTest("C:\\Users\\ccc0032\\Documents\\Refined.pnt", ios::in | ios::binary);
		while (ReadTest) {
			double X;
			double Y;
			double Z;
			ReadTest.read((char*)&X, sizeof(double));
			ReadTest.read((char*)&Y, sizeof(double));
			ReadTest.read((char*)&Z, sizeof(double));
			std::cout << "X: " << X << "\tY: " << Y << "\tZ: " << Z << "\n";
		}
		ReadTest.close();
	}
	else {
		double Coords[3] = {};
		ifstream ReadTest("C:\\Users\\ccc0032\\Documents\\Refined.pnt", ios::in | ios::binary);
		//ofstream OutFile;
		//OutFile.open("C:\\Users\\ccc0032\\Documents\\Load.obj", ios::out);
		//OutFile << "o Block\n";
		//OutFile.precision(10);
		double MinPointx = 100000000;
		double MaxPointx = -100000000;

		double MinPointy = 100000000;
		double MaxPointy = -100000000;

		double MinPointz = 100000000;
		double MaxPointz = -100000000;

		while (ReadTest) {
			ReadTest.read((char*)&Coords, sizeof(double) * 3);
			//OutFile << "v " << Coords[0] << " " << Coords[1] << " " << Coords[2] << "\n";

			if (Coords[0] < MinPointx) { MinPointx = Coords[0]; }

			if (Coords[1] < MinPointy) { MinPointy = Coords[1]; }

			if (Coords[2] < MinPointz) { MinPointz = Coords[2]; }

			if (Coords[0] > MaxPointx) { MaxPointx = Coords[0]; }

			if (Coords[1] > MaxPointy) { MaxPointy = Coords[1]; }

			if (Coords[2] > MaxPointz) { MaxPointz = Coords[2]; }

			//OutFile << "v " << Coords[0] << " " << Coords[1] << " " << Coords[2] << "\n";

		}
		cout.precision(10);
		cout << "MinPointX: " << MinPointx << "\tMaxPointX: " << MaxPointx << "\n";
		cout << "MinPointY: " << MinPointy << "\tMaxPointY: " << MaxPointy << "\n";
		ReadTest.close();
		//OutFile.close();

	}
	

}
