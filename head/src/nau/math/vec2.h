#ifndef NAU_VECTOR2_H
#define NAU_VECTOR2_H

#include "nau/math/data.h"
#include "nau/math/utils.h"

#include <string>

namespace nau
{
	namespace math
	{
		template <typename T>
		class vector2: public Data {

		public:

			T x, y;

			vector2() : x(0), y(0) {};
			vector2(T x, T y) : x(x), y(y) {};
			vector2(T v) : x(v), y(v) {};
			vector2(const vector2 &v) : x(v.x), y(v.y) {};
			vector2(const T* values) {
				x = values[0];
				y = values[1];
			}
			
			~vector2() {};

			const vector2&
				vector2::operator =(const vector2 &v) {

				if (this != &v) {
					x = v.x;
					y = v.y;
				}
				return *this;
			};


			Data *clone() {
				return new vector2(*this);
			}

			void *getPtr() {
				return &x;
			}

			void
				copy(const vector2 &v) {

				x = v.x;
				y = v.y;
			};

			//vector2 *
			//	clone() const {

			//	return new vector(*this);
			//};

			void
				set(T xx, T yy) {
				x = xx;
				y = yy;
			};

			void
				set(T *values) {
				x = values[0];
				y = values[1];
			};

			void
				set(vector2* aVec) {
				x = aVec->x;
				y = aVec->y;
			};

			void
				set(const vector2& aVec) {
				x = aVec.x;
				y = aVec.y;
			};

			const vector2&
				operator += (const vector2 &v)	{

				x += v.x;
				y += v.y;
				return *this;
			};

			const vector2&
				operator -= (const vector2 &v) {

				x -= v.x;
				y -= v.y;
				return (*this);
			};

			const vector2 &
				operator - (void) {

				x = -x;
				y = -y;
				return (*this);
			};

			const vector2&
				operator *=(T t) {

				x *= t;
				y *= t;
				return *this;
			};

			const vector2&
				operator /=(T t) {

				x /= t;
				y /= t;
				return *this;
			};

			bool
				operator == (const vector2 &v) const {

				return equals(v);
			};

			bool
				operator > (const vector2 &v) const {

				if (x > v.x && y > v.y)
					return true;
				else
					return false;
			}

			bool
				operator < (const vector2 &v) const {

				if (x < v.x && y < v.y)
					return true;
				else
					return false;
			}

			bool
				operator != (const vector2 &v) const {

				return !equals(v);
			};


			void
				add(const vector2 &v) {
				x += v.x;
				y += v.y;
			};

			void
				scale(T a) {

				x *= a;
				y *= a;
			};

			float
				length() const {

				return sqrtf(x*x + y*y);
			};

			void
				normalize() {

				float m = length();
				if (m <= FLT_EPSILON) {
					m = 1;
				}
				x /= m;
				y /= m;
			};

			float
				dot(const vector2 &v) const {

				return (x*v.x + y*v.y);
			};

			const vector2
				cross(const vector2 &v) const {

				vector2 result;

				result.x = 0.0f;
				result.y = 0.0f;

				return result;
			};

			bool
				between(const vector2 &v1, const vector2 &v2) {

				if (x < v1.x || x > v2.x)
					return false;
				if (y < v1.y || y > v2.y)
					return false;

				return true;
			};

			bool
				equals(const vector2 &v, float tolerance = -1.0f) const {
				return (FloatEqual((float)x, (float)v.x, tolerance) && FloatEqual((float)y, (float)v.y, tolerance));
			};

			std::string 
				toString() {

				return  "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
			};
		};

		typedef vector2<float> vec2;
		typedef vector2<int> ivec2;
		typedef vector2 < unsigned int >  uivec2;
		typedef vector2<bool> bvec2;
		typedef vector2<double> dvec2;
	};
};


#endif 
