#ifndef DATALOADER_UTILS
#define DATALOADER_UTILS

#include<iostream>
#include<vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace RVizDataLoader 
{

namespace Utils
{
    template<typename T>
    class Vec3
    {
    public:
        Vec3(): x_val(T{}), y_val(T{}), z_val(T{}) {}
        Vec3(T x, T y, T z): x_val(x), y_val(y), z_val(z) {}
        Vec3(std::vector<T> vec3)
        {
            assert(vec3.size() == 3);
            Vec3<T>::x_val = vec3[0];
            Vec3<T>::y_val = vec3[1];
            Vec3<T>::z_val = vec3[2];
        }

        virtual void update(T x, T y, T z)
        {
            x_val = x;
            y_val = y;
            z_val = z;
        }

        virtual void update(std::vector<T> vec)
        {
            x_val = vec[0];
            y_val = vec[1];
            z_val = vec[2];
        }

        virtual std::vector<T> asVector()
        {
            std::vector<T> vec{ x_val, y_val, z_val };
            return vec;
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec3<T>& data)
        {
            os << "(" << data.x() << ", " << data.y() << ", " << data.z() << ")";
            return os;
        }

        T x() const { return x_val; };
        T y() const { return y_val; };
        T z() const { return z_val; };

        T roll() const { return x(); };
        T pitch() const { return y(); };
        T yaw() const { return z(); };

        T r() const { return x(); }
        T g() const { return y(); }
        T b() const { return z(); }

    protected:
        T x_val;
        T y_val;
        T z_val;
    };

    template<typename T>
    class Vec4 : public Vec3<T>
    {
    public:
        Vec4(): Vec3<T>(), w_val(T{}) {}
        Vec4(T x, T y, T z, T w): Vec3<T>(x, y, z), w_val(w) {}
        Vec4(std::vector<T> vec4)
        {
            assert(vec4.size() == 4);
            Vec3<T>::x_val = vec4[0];
            Vec3<T>::y_val = vec4[1];
            Vec3<T>::z_val = vec4[2];
            w_val = vec4[3];
        }

        void update(T x, T y, T z, T w)
        {
            Vec3<T>::x_val = x;
            Vec3<T>::y_val = y;
            Vec3<T>::z_val = z;
            w_val = w;
        }

        virtual void update(std::vector<T> vec)
        {
            Vec3<T>::x_val = vec[0];
            Vec3<T>::y_val = vec[1];
            Vec3<T>::z_val = vec[2];
            w_val = vec[3];
        }

        virtual std::vector<T> asVector()
        {
            std::vector<T> vec{ Vec3<T>::x_val, Vec3<T>::y_val, Vec3<T>::z_val, w_val };
            return vec;
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec4<T>& data)
        {
            os << "(" << data.x() << ", " << data.y() << ", " << data.z() << ", " << data.w() << ")";
            return os;
        }

        T w() const { return w_val; };
        T a() const { return w(); }

    protected:
        T w_val;
    };

    template<typename T>
    class Pose
    {
    public:
        Pose(){}

        Pose(T x, T y, T z, T roll, T pitch, T yaw): Pose()
        {
            update(x, y, z, roll, pitch, yaw);
        }

        void updatePosition(T x, T y, T z)
        {
            position.update(x, y, z);
        }

        void updatePosition(Vec3<T> pos)
        {
            position.update(pos.x(), pos.y(), pos.z());
        }

        void updatePosition(std::vector<T> pos)
        {
            position.update(pos[0], pos[1], pos[2]);
        } 

        void updateOrientation(T roll, T pitch, T yaw)
        {
            orientation.update(roll, pitch, yaw);
        }

        void updateOrientation(Vec3<T> ori)
        {
            orientation.update(ori.roll(), ori.pitch(), ori.yaw());
        }

        void updateOrientation(std::vector<T> ori)
        {
            orientation.update(ori[0], ori[1], ori[2]);
        } 

        void update(T x, T y, T z, T roll, T pitch, T yaw) 
        {
            updatePosition(x, y, z);
            updateOrientation(roll, pitch, yaw);
        }

        void update(Vec3<T> pos, Vec3<T> ori) 
        {
            updatePosition(pos);
            updateOrientation(ori);
        }

        void update(std::vector<T> pos, std::vector<T> ori) 
        {
            updatePosition(pos);
            updateOrientation(ori);
        }

        friend std::ostream& operator<<(std::ostream& os, const Pose<T>& pose)
        {
            os << "Position" << pose.position << "; Orientation" << pose.orientation;
            return os;
        }

        Vec3<T> position;
        Vec3<T> orientation;
    };

    template<typename T>
    using Vec3Array = std::vector<Vec3<T>>;

    template<typename T>
    using Vec4Array = std::vector<Vec4<T>>;

    template<typename T>
    using PoseArray = std::vector<Pose<T>>;

    static tf2::Quaternion toTf2Quaternion(Vec3<double> orientationRPY)
    {
        tf2::Quaternion quat;
        quat.setRPY(orientationRPY.roll(), orientationRPY.pitch(), orientationRPY.yaw());
        quat.normalize();

        return quat;
    }

    static Vec4<double> toQuaternion(Vec3<double> orientationRPY)
    {
        tf2::Quaternion quat = toTf2Quaternion(orientationRPY);
        return Vec4<double>(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    static Vec3<double> toRPY(Vec4<double> orientationQuat)
    {
        tf2::Quaternion quat(orientationQuat.x(), orientationQuat.y(), 
                             orientationQuat.z(), orientationQuat.w());
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        return Vec3<double>(roll, pitch, yaw);
    }
}

}

#endif //DATALOADER_UTILS
