/*****************************************************************************\
*                                                                           *
* File(s): CForgeMath.h and CForgeMath.cpp                                  *
*                                                                           *
* Content: Utility methods for math stuff.                                  *
*                                                                           *
*                                                                           *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_CFORGEMATH_H__
#define __CFORGE_CFORGEMATH_H__

#include <chrono>
#include "../Core/CForgeObject.h"

namespace CForge {

	/**
	* \brief Utility class offering math support methods.
	* \ingroup Math
	* 
	* \todo Test whether all random method works correctly/plausible.
	* \todo Change pass by value to pass by reference for pointers.
	*/
	class CFORGE_API CForgeMath : public CForgeObject {
	public:
		/**
		* \brief Computes sign of the given parameter.
		* 
		* \param[in] arg Value to dervice sign from.
		* \return 1 for positive values, -1 for negative values, 0 otherwise.
		*/
		template<typename T>
		static T sign(const T arg) {
			if (arg > T(0)) return T(1);
			else if (arg < T(0)) return (T)(-1);
			return T(0);
		}//sign

		/**
		* \brief Basic linear interpolation. \f$ y = a + t \cdot (b - a) \f$ 
		* 
		* \param[in] a Value a.
		* \param[in] b Value b.
		* \param[in] t Interpolation parameter \f$\in [0,1] \f$
		*/
		template<typename T, typename P>
		static T lerp(T a, T b, P t) {
			return T(a + t * (b - a));
		}//lerp

		/**
		* \brief Computes the next power of two for a given value.
		* 
		* \param[in] Value Input value.
		* \return Next power of two to value.
		*/
		template<typename T>
		static T nextPowerOfTwo(const T Value) {
			T a = T(std::log2(Value));
			return (std::pow(T(2), a) == Value) ? Value : std::pow(T(2), a + T(1));
		}//nextPowerOfTwo

		/**
		* \brief Returns a random value in a specific range.
		* 
		* \param[in] Lower Lower limit.
		* \param[in] Upper Upper limit.
		* \return Value \f$ \in [Lower, Upper] \f$
		*/
		template<typename T>
		static T randRange(T Lower, T Upper) {
			long double R = (long double)(rand()) / (long double)(randMax());
			return lerp(Lower, Upper, R);
		}//randRange

		/**
		* \brief Returns positive random value. Only integer template parameters supported.
		* 
		* \return Random positive number.
		*/
		template<typename T>
		static T rand(void) {
			return (rand() % std::numeric_limits<T>::max());
		}//rand

		/**
		* \brief Return a pseudo random number. Internally the generator is based on XOrShift.
		* 
		* See https://en.wikipedia.org/wiki/Xorshift for an introduction. Use the randSeed method to set an initial value.
		* \return Pseudo random number.
		*/
		static uint64_t rand(void);

		/**
		* \brief Set the seed value for the random number generator.
		* 
		* \param[in] Seed New seed value.
		*/
		static void randSeed(uint64_t Seed);

		/**
		* \brief Returns the maximum value the random number generator can produce.
		* 
		* \return Maximum value the random number generator can produce.
		*/
		static uint64_t randMax(void) {
			return std::numeric_limits<uint64_t>::max();
		}//randMax

		/**
		* \brief Converts degree value to radians: \f$ f(x) = x \cdot \frac{\pi}{180} \f$
		* 
		* \param[in] Value in degree.
		* \returns Value in radian. 
		*/
		template<typename T>
		static T degToRad(T Deg) {
			return Deg * T(EIGEN_PI) / T(180);
		}//degToRad

		/**
		* \brief Converts degree to radian: \f$ f(x) = x \cdot \frac{180}{\pi} \f$
		* 
		* \param[in] Value in radians.
		* \return Valid in degree.
		*/
		template<typename T>
		static T radToDeg(T Rad) {
			return Rad * T(180) / T(EIGEN_PI);
		}//radToDeg

		/**
		* \brief Creates a perspective projection matrix.
		* 
		* \param[in] Width The width of the viewport.
		* \param[in] Height The height of the viewport.
		* \param[in] FieldOfView Field of view in radians.
		* \param[in] Near Distance of the near clipping plane.
		* \param[in] Far Distance of the far clipping plane.
		* \return Perspective projection matrix.
		*/
		static Eigen::Matrix4f perspectiveProjection(uint32_t Width, uint32_t Height, float FieldOfView, float Near, float Far);

		/**
		* \brief Creates a perspective projection matrix.
		* 
		* \param[in] Left Near clipping plane left value.
		* \param[in] Right Near clipping plane right value.
		* \param[in] Bottom Near clipping plane bottom value.
		* \param[in] Top Near clipping plane top value.
		* \param[in] Near Distance of the near clipping plane.
		* \param[in] Far Distance of the far clipping plane
		* \return Perspective projection matrix.
		*/
		static Eigen::Matrix4f perspectiveProjection(float Left, float Right, float Bottom, float Top, float Near, float Far);

		/**
		* \brief Creates an orthographic projection matrix.
		* 
		* \param[in] Left Near clipping plane left value.
		* \param[in] Right Near clipping plane right value.
		* \param[in] Bottom Near clipping plane bottom value.
		* \param[in] Top Near clipping plane top value.
		* \param[in] Near Distance of near clipping plane.
		* \param[in] Far Distance of far clipping plane.
		* \return Orthographic projection matrix.
		*/
		static Eigen::Matrix4f orthographicProjection(float Left, float Right, float Bottom, float Top, float Near, float Far);

		/**
		* \brief Creates an orthographic projection matrix. This method assumes symmetry of the plane.
		* 
		* \param[in] Right Near clipping plane right value. Left value will be -Right.
		* \param[in] Top Near clipping plane top value. Bottom value will be -Top.
		* \param[in] Near Distance of the near clipping plane.
		* \param[in] Far Distance of the far clipping plane.
		* \return Orthographic projection matrix.
		*/
		static Eigen::Matrix4f orthographicProjection(float Right, float Top, float Near, float Far);

		/**
		* \brief Creates stereo frustums, i.e. perspective projection matrix for the left and right eye.
		* 
		* \param[in] Width The width of the viewport.
		* \param[in] Height The height of the viewport.
		* \param[in] Near Distance of the near clipping plane.
		* \param[in] Far Distance of the far clipping plane.
		* \param[in] FOV Field of view in radians.
		* \param[in] FocalLength The focal length.
		* \param[in] EyeSep The eye separation.
		* \param[out] LeftEye Projection matrix of the left eye will be stored here.
		* \param[out] RightEye Projection matrix of the right eye will be stored here.
		*/
		static void stereoFrustums(uint32_t Width, uint32_t Height, float Near, float Far, float FOV, float FocalLength, float EyeSep, Eigen::Matrix4f &LeftEye, Eigen::Matrix4f &RightEye);

		/**
		* \brief Creates a camera (view) matrix using a camera position and a target position to look at.
		* 
		* \param[in] Position Camera's position.
		* \param[in] Target Point to look at.
		* \param[in] Up A vector that points up in the camera's coordinate system.
		* \return Camera matrix aka view matrix.
		*/
		static Eigen::Matrix4f lookAt(Eigen::Vector3f Position, Eigen::Vector3f Target, Eigen::Vector3f Up = Eigen::Vector3f::UnitY());

		/**
		* \brief Creates a 4x4 rotation matrix from a quaternion.
		* 
		* \param[in] Rot The quaternion representing a rotation.
		* \return Rotations matrix.
		*/
		static Eigen::Matrix4f rotationMatrix(Eigen::Quaternionf Rot);

		/**
		* \brief Creates a 4x4 rotation matrix from angle axis
		* 
		* \param[in] angle in degrees.
		* \param[in] axis needs to be normalized.
		* \return Rotations matrix.
		*/
		static Eigen::Matrix4f rotationMatrix(float angle, Eigen::Vector3f axis);

		/**
		* \brief Creates a 4x4 translation matrix from a vector.
		* 
		* \param[in] Trans The translation.
		* \return Translation matrix.
		*/
		static Eigen::Matrix4f translationMatrix(Eigen::Vector3f Trans);

		/**
		* \brief Creates a scaling matrix from a scale vector (x,y,z).
		* 
		* \param[in] Scale Scaling factors in x,y, and z.
		* \return Scaling matrix.
		*/
		static Eigen::Matrix4f scaleMatrix(Eigen::Vector3f Scale);

		/**
		* \brief Computes a rotation matrix that aligns the source vector into the target vector.
		* 
		* \param[in] Source Source vector.
		* \param[in] Target Target vector.
		* \return Rotations matrix that rotates Source into Target.
		*/
		static Eigen::Matrix3f alignVectors(const Eigen::Vector3f Source, const Eigen::Vector3f Target);

		/**
		* \brief Implementation of equirectangular projection.
		* 
		* \param[in] Pos Position in 3D.
		* \return Mapping to the 2D domain.
		*/
		static Eigen::Vector3f equirectangularMapping(const Eigen::Vector3f Pos);

		/**
		* \brief Implementation of equal area mapping.
		* 
		* \param[in] Pos Position in 3D.
		* \return Mapping to the 2D domain.
		*/
		static Eigen::Vector3f equalAreaMapping(const Eigen::Vector3f Pos);

		/**
		* \brief Constructor
		*/
		CForgeMath(void);

		/**
		* \brief Destructor
		*/
		~CForgeMath(void);

	protected:
		static uint64_t m_RndState;		///< Current value of the random number generator.
	};//CForgeMath

}//name space

#endif 