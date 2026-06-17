#include <iostream>
#include <iomanip>
#include <cmath>
#include <variant>
#include <algorithm>

class Vector3D {
    double X;
    double Y;
    double Z;

public:
    Vector3D(double x = 0.0, double y = 0.0, double z = 0.0) : X(x), Y(y), Z(z) {}

    double GetX() const { return X; }
    double GetY() const { return Y; }
    double GetZ() const { return Z; }

    friend std::istream& operator>>(std::istream& in, Vector3D& obj) {
        in >> obj.X >> obj.Y >> obj.Z;
        return in;
    }

    friend std::ostream& operator<<(std::ostream& out, const Vector3D& obj) {
        out << std::fixed << std::setprecision(3) << obj.X << " " << obj.Y << " " << obj.Z;
        return out;
    }

    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(X + other.X, Y + other.Y, Z + other.Z);
    }

    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(X - other.X, Y - other.Y, Z - other.Z);
    }

    Vector3D operator*(double scalar) const {
        return Vector3D(X * scalar, Y * scalar, Z * scalar);
    }

    // Скалярное произведение
    double operator*(const Vector3D& other) const {
        return X * other.X + Y * other.Y + Z * other.Z;
    }

    // Векторное произведение
    Vector3D Cross(const Vector3D& other) const {
        return Vector3D(
            Y * other.Z - Z * other.Y,
            Z * other.X - X * other.Z,
            X * other.Y - Y * other.X
        );
    }

    // Квадрат длины используется для оптимизации вычислений
    double SquaredLength() const {
        return X * X + Y * Y + Z * Z;
    }

    double Length() const {
        return std::sqrt(SquaredLength());
    }

    bool IsEqual(const Vector3D& other, double eps) const {
        return SquaredLength() <= eps;
    }
};

class Segment3D {
    Vector3D start;
    Vector3D end;

public:
    Segment3D() = default;
    Segment3D(const Vector3D& s, const Vector3D& e) : start(s), end(e) {}

    const Vector3D& GetStart() const { return start; }
    const Vector3D& GetEnd() const { return end; }

    Vector3D GetDirection() const { return end - start; }
    double SquaredLength() const { return GetDirection().SquaredLength(); }
};

Segment3D ReadSegment() {
    Vector3D start, end;
    std::cin >> start >> end;
    return Segment3D(start, end);
}

std::variant<std::monostate, Vector3D, Segment3D> Intersect(const Segment3D& A, const Segment3D& B, double eps = 1e-9) {
    Vector3D dirA = A.GetDirection();
    Vector3D dirB = B.GetDirection();
    Vector3D AB = A.GetStart() - B.GetStart();

    double a = dirA.SquaredLength();
    double c = dirB.SquaredLength();

    // Защита от вырожденных отрезков (длина равна нулю)
    if (a <= eps * eps || c <= eps * eps) {
        if (a <= eps * eps && c <= eps * eps && (A.GetStart() - B.GetStart()).SquaredLength() <= eps * eps) {
            return A.GetStart();
        }
        return std::monostate{};
    }

    double b = dirA * dirB;
    double d = dirA * AB;
    double e = dirB * AB;

    double denominator = a * c - b * b;

    // Проверка на параллельность
    if (denominator * denominator <= eps) {

        // Проверка: лежат ли отрезки на одной прямой (расстояние от B до прямой A)
        Vector3D perp = AB - dirA * (d / a);
        if (perp.SquaredLength() > eps) {
            return std::monostate{};
        }

        // Отрезки коллинеарны. Проецируем концы отрезка B на прямую A
        double tB_start = -d / a;
        double tB_end = (dirA * (B.GetEnd() - A.GetStart())) / a;

        if (tB_start > tB_end) std::swap(tB_start, tB_end);

        double overlapStart = std::max(0.0, tB_start);
        double overlapEnd = std::min(1.0, tB_end);

        if (overlapStart > overlapEnd + eps) {
            return std::monostate{};
        }

        if (std::abs(overlapStart - overlapEnd) <= eps) {
            return A.GetStart() + dirA * overlapStart;
        }

        return Segment3D(
            A.GetStart() + dirA * overlapStart,
            A.GetStart() + dirA * overlapEnd
        );
    }

    // Случай скрещивающихся или пересекающихся прямых
    double t = (b * e - c * d) / denominator;
    double s = (a * e - b * d) / denominator;

    // Проверка попадания точек в границы отрезков
    if (t < -eps || t > 1.0 + eps || s < -eps || s > 1.0 + eps) {
        return std::monostate{};
    }

    Vector3D pointA = A.GetStart() + dirA * t;
    Vector3D pointB = B.GetStart() + dirB * s;

    // Проверка кратчайшего расстояния между скрещивающимися прямыми
    if ((pointA - pointB).SquaredLength() > eps) {
        return std::monostate{};
    }

    return (pointA + pointB) * 0.5;
}

void PrintResult(const std::variant<std::monostate, Vector3D, Segment3D>& result) {
    struct Visitor {
        void operator()(std::monostate) const {
            std::cout << "Segments do not intersect.\n";
        }
        void operator()(const Vector3D& p) const {
            std::cout << "Intersection point: " << p << "\n";
        }
        void operator()(const Segment3D& seg) const {
            std::cout << "Segments overlap from "
                << seg.GetStart() << " to " << seg.GetEnd() << "\n";
        }
    };
    std::visit(Visitor{}, result);
}

int main() {
    std::cout << "Enter segment A (start x y z, end x y z):\n";
    Segment3D segmentA = ReadSegment();

    std::cout << "Enter segment B (start x y z, end x y z):\n";
    Segment3D segmentB = ReadSegment();

    std::cout << "Enter epsilon value (e.g. 1e-9): ";
    double eps;
    std::cin >> eps;

    std::variant<std::monostate, Vector3D, Segment3D> result = Intersect(segmentA, segmentB, eps);
    PrintResult(result);

    return 0;
}