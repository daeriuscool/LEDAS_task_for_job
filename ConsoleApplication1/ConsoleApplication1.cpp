#include <iostream>
#include <iomanip>
#include <utility>

using namespace std;

class Vector3D {
	double X;
	double Y;
	double Z;

public: 

	Vector3D() {
		X = 0;
		Y = 0;
		Z = 0;
	}

	Vector3D(double x, double y, double z) {
		X = x;
		Y = y;
		Z = z;
	}

	// Copy constructor
	Vector3D(const Vector3D& other) : X(other.X), Y(other.Y), Z(other.Z) {}

	// Copy assignment
	Vector3D& operator=(const Vector3D& other) {
		if (this != &other) {
			X = other.X;
			Y = other.Y;
			Z = other.Z;
		}
		return *this;
	}

	// Move constructor
	Vector3D(Vector3D&& other) noexcept : X(other.X), Y(other.Y), Z(other.Z) {
		other.X = other.Y = other.Z = 0;
	}

	// Move assignment
	Vector3D& operator=(Vector3D&& other) noexcept {
		if (this != &other) {
			X = other.X;
			Y = other.Y;
			Z = other.Z;
			other.X = other.Y = other.Z = 0;
		}
		return *this;
	}

	// Overloading operators for Vector3D

	friend istream& operator>>(istream& in, Vector3D& obj) {
		cin >> obj.X >> obj.Y >> obj.Z;
		return in;
	}

	friend ostream& operator<<(ostream& out, const Vector3D& obj) {
		out << fixed << setprecision(3) << obj.X << " " << obj.Y << " " << obj.Z;
		return out;
	}

	Vector3D operator+(const Vector3D& other) {
		return Vector3D(X + other.X, Y + other.Y, Z + other.Z);
	}

	Vector3D operator-(const Vector3D& other) {
		return Vector3D(X - other.X, Y - other.Y, Z - other.Z);
	}

	Vector3D operator*(double scalar) {
		return Vector3D(X * scalar, Y * scalar, Z * scalar);
	}

	double operator*(const Vector3D& other) {
		return X * other.X + Y * other.Y + Z * other.Z;
	}

	Vector3D Cross(const Vector3D& other) {
		return Vector3D(
			Y * other.Z - Z * other.Y,
			Z * other.X - X * other.Z,
			X * other.Y - Y * other.X
		);
	}

	bool operator==(const Vector3D& other) {
		return X == other.X && Y == other.Y && Z == other.Z;
	}

	double Length() {
		return sqrt(X * X + Y * Y + Z * Z);
	}
};

class Segment3D {
	Vector3D start;
	Vector3D end;

public:
	Segment3D() {
		start = Vector3D();
		end = Vector3D();
	}
	Segment3D(const Vector3D& s, const Vector3D& e) {
		start = s;
		end = e;
	}

	// Copy constructor
	Segment3D(const Segment3D& other) : start(other.start), end(other.end) {}

	// Copy assignment
	Segment3D& operator=(const Segment3D& other) {
		if (this != &other) {
			start = other.start;
			end = other.end;
		}
		return *this;
	}

	// Move constructor
	Segment3D(Segment3D&& other) noexcept : start(std::move(other.start)), end(std::move(other.end)) {}

	// Move assignment
	Segment3D& operator=(Segment3D&& other) noexcept {
		if (this != &other) {
			start = std::move(other.start);
			end = std::move(other.end);
		}
		return *this;
	}

	Vector3D getStart() {
		return start;
	}

	Vector3D getEnd() {
		return end;
	}

	void enterSegment() {
		//cout << "Enter start point (x y z): ";
		cin >> start;
		//cout << "Enter end point (x y z): ";
		cin >> end;
	}

};

// Method to read a segment from user input
Segment3D readSegment() {
	Segment3D segment;
	segment.enterSegment();
	return segment;
}

// Function to find the intersection of two segments in 3D space
void Intersect(Segment3D A, Segment3D B, double eps=1e-10) {

	Vector3D directionA = A.getEnd() - A.getStart();
	Vector3D Astart = A.getStart();
	Vector3D directionB = B.getEnd() - B.getStart();
	Vector3D Bstart = B.getStart();
	Vector3D AB = Astart - Bstart;


	// Coefficients for the system of equations
	double a = directionA * directionA;
	double b = directionA * directionB;
	double c = directionB * directionB;
	double d = directionA * AB;
	double e = directionB * AB;
	// Determinant of the system
	double denominator = a * c - b * b;

	// Check if lines are parallel (denominator close to zero)
	if (fabs(denominator) <= eps) {
		double tA = -d / a;
		Vector3D p_proj = Astart + directionA * tA;
		Vector3D dist_vec = Bstart - p_proj;

		// If the distance from the projected point to the start of segment B is greater than eps, they are parallel but not coincident
		if (dist_vec.Length() > eps) {
			cout << "Lines are parallel but not coincident.";
			return;
		}
		else
		{
			// Lines are collinear, check for overlap
			double tB = (b - d) / a;

			double tMin = min(tA, tB);
			double tMax = max(tA, tB);

			double overlapStart = max(0.0, tMin);
			double overlapEnd = min(1.0, tMax);

			// Check if there is an overlap
			if (overlapStart > overlapEnd + eps) {
				cout << "Segments are on the one line but do not overlap.";
				return;
			}
			else if (fabs(overlapStart - overlapEnd) <= eps) {
				cout << "Segments are on the one line and touch at point: "
					<< Astart + directionA * overlapStart;
				return;
			}
			else {
				cout << "Segments are on the one line and overlap from "
					<< Astart + directionA * overlapStart << " to "
					<< Astart + directionA * overlapEnd;
				return;
			}
		}
	}

	// Calculate the parameters t and s for the closest points on the lines
	double t = (b * e - c * d) / denominator;
	double s = (a * e - b * d) / denominator;

	// Check if the closest points are within the segments
	if (t < 0.0 || t > 1.0 || s < 0.0 || s > 1.0) {
		cout << "Segments do not intersect.";
		return;
	}

	Vector3D intersectionPointA = Astart + directionA * t;
	Vector3D intersectionPointB = Bstart + directionB * s;

	// Check if the intersection points are close enough to be considered the same point
	Vector3D diff = intersectionPointA - intersectionPointB;

	if (diff.Length() > eps) {
		cout << "Segments do not intersect (with eps = " << fixed << setprecision(3) << eps << ")";
		return;
	}

	// If we reach this point, the segments intersect at a single point
	cout << "Intersection point: " << (intersectionPointA + intersectionPointB) * 0.5;
}

int main() {
	
	Segment3D segmentA = readSegment(), segmentB=readSegment();

	cout << "Enter epsilon value (default 1e-10): ";
	double eps;
	cin >> eps;

	Intersect(segmentA, segmentB, eps);

	return 0;
}