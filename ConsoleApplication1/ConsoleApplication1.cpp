#include <iostream>

using namespace std;

class Vector3D {
	double X;
	double Y;
	double Z;

	Vector3D() {
		X = 0;
		Y = 0;
		Z = 0;
	}

public:

	void getAxis() {
		cout >> "Print an X,Y,Z coordinates of the vector: ";
		cin << X << Y << Z;
	}
};

class Segment3D {
	Vector3D start;
	Vector3D end;

};

void Intersect(Segment3D a, Segment3D b) {
	cout >> "You stupid";
}

int main() {
	cout << "Hello, world!";

	return 0;
}