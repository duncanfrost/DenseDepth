// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//#define USE_EIGEN
#pragma once
#include <ORUtils/MathTypes.h>
#include <ORUtils/Vector.h>
#include <ORUtils/SE3Pose.h>
#ifdef USE_EIGEN
    #include <Eigen/Core>
#endif

namespace ORUtils {

    class SE3f : public SE3Pose
    {
    public:

        SE3f(SE3Pose in)
        {
            this->SetM(in.GetM());
        }

        SE3f(void) { this->SetFrom(0, 0, 0, 0, 0, 0); }
        SE3f(const Matrix3<float> &R, const Vector3<float> &t)
        {this->SetRT(R,t);}
        SE3f(const Matrix4<float> & src) { this->SetM(src); }

        Vector3f translation() { return GetT(); }
        Matrix3f rotationMatrix() { return GetR();}
        void setRotationMatrix(Matrix3f in) { setRotationMatrix(in);}
        SE3f inverse() { return SE3Pose(GetInvM());}
        inline
        const Vector3f operator*(const Vector3f & p) const {
          return GetR()*p + GetT();}

        const SE3f operator*(const SE3f& other) const {
          Matrix4f m1 = GetM();
          Matrix4f m2 = other.GetM();
          SE3f result(m1*m2);
          return result;
        }

        static SE3f exp(const Vector6<float>& tangent)
        {
            return (SE3f)SE3Pose(tangent);
        }

        typedef Vector6f Tangent;
    };
}





namespace DenseMono {
    typedef ORUtils::SE3f SE3f;
	typedef Vector2f Vector2f;
	typedef Vector3f Vector3f;
	typedef Matrix3f Matrix3f;
	typedef Matrix4f Matrix4f;
	typedef Vector4f Vector4f;
}

#define DOT(a,b) ORUtils::dot(a,b)
#define TRANS(M) M.t()
#define SET_TRANS(P,t) P.SetT(t);
#define GET_ROW(M,i) M.getRow(i)
