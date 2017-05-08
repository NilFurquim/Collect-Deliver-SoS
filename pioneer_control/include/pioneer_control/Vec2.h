#ifndef _VEC2_H
#define _VEC2_H_

template <class T>
class Vec2
{
	public:
		T x, y;
		Vec2(){};
		Vec2(T x, T y) : x (x), y(y) {};
		Vec2(const Vec2& v) : x(v.x), y(v.y) {};

		Vec2 &operator=(const Vec2& v)
		{
			x = v.x;
			y = v.y;
			return *this;
		};
		
		Vec2 operator-(const Vec2& v)
		{ return Vec2(x - v.x, y - v.y); };

		Vec2 operator+(const Vec2& v)
		{ return Vec2(x + v.x, y + v.y); };

		Vec2 &operator+=(const Vec2& v)
		{
			x += v.x;
			y += v.y;
			return *this;
		};
		
		Vec2 &operator-=(const Vec2& v)
		{
			x -= v.x;
			y -= v.y;
			return *this;
		};

		double operator^(const Vec2& v)
		{ return x * v.y - y * v.x; }

		double operator*(const Vec2& v)
		{ return x * v.x + y * v.y; }

		bool operator<=(const Vec2& v) const
		{ return (x <= v.x && y <= v.y); }

		bool operator<(const Vec2& v) const
		{ return (x < v.x && y < v.y); }

		bool operator>=(const Vec2& v) const
		{ return (x >= v.x && y >= v.y); }

		bool operator>(const Vec2& v) const
		{ return (x > v.x && y > v.y); }

		bool operator==(const Vec2& v) const
		{ return (x == v.x && y == v.y); }

		bool operator!=(const Vec2& v) const
		{ return (x != v.x || y != v.y); }

		void rotate(const double theta) {
			double c = cos(theta);
			double s = sin(theta);
			double tx = x * c - y * s;
			double ty = x * s + y * c;
			x = tx;
			y = ty;
		}

};

typedef Vec2<int> Vec2i;
typedef Vec2<double> Vec2d;
#endif
