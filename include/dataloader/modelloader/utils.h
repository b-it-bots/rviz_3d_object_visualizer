/*
 * MIT License
 * 
 * Copyright (c) 2020 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/**
  File: utils.h
  Purpose: Custom storage classes and utility functions for vector and pose data
  @author Sushant Vijay Chavan
  @version 1.0 16/10/20
*/

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
        Vec3(): x_val_(T{}), y_val_(T{}), z_val_(T{}) {}
        Vec3(T x, T y, T z): x_val_(x), y_val_(y), z_val_(z) {}
        Vec3(std::vector<T> vec3)
        {
            assert(vec3.size() == 3);
            Vec3<T>::x_val_ = vec3[0];
            Vec3<T>::y_val_ = vec3[1];
            Vec3<T>::z_val_ = vec3[2];
        }

        virtual void update(T x, T y, T z)
        {
            x_val_ = x;
            y_val_ = y;
            z_val_ = z;
        }

        virtual void update(std::vector<T> vec)
        {
            x_val_ = vec[0];
            y_val_ = vec[1];
            z_val_ = vec[2];
        }

        virtual std::vector<T> asVector()
        {
            std::vector<T> vec{ x_val_, y_val_, z_val_ };
            return vec;
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec3<T>& data)
        {
            os << "(" << data.x() << ", " << data.y() << ", " << data.z() << ")";
            return os;
        }

        T x() const { return x_val_; };
        T y() const { return y_val_; };
        T z() const { return z_val_; };

        T roll() const { return x(); };
        T pitch() const { return y(); };
        T yaw() const { return z(); };

        T r() const { return x(); }
        T g() const { return y(); }
        T b() const { return z(); }

    protected:
        T x_val_;
        T y_val_;
        T z_val_;
    };

    template<typename T>
    class Vec4 : public Vec3<T>
    {
    public:
        Vec4(): Vec3<T>(), w_val_(T{}) {}
        Vec4(T x, T y, T z, T w): Vec3<T>(x, y, z), w_val_(w) {}
        Vec4(std::vector<T> vec4)
        {
            assert(vec4.size() == 4);
            Vec3<T>::x_val_ = vec4[0];
            Vec3<T>::y_val_ = vec4[1];
            Vec3<T>::z_val_ = vec4[2];
            w_val_ = vec4[3];
        }

        void update(T x, T y, T z, T w)
        {
            Vec3<T>::x_val_ = x;
            Vec3<T>::y_val_ = y;
            Vec3<T>::z_val_ = z;
            w_val_ = w;
        }

        virtual void update(std::vector<T> vec)
        {
            Vec3<T>::x_val_ = vec[0];
            Vec3<T>::y_val_ = vec[1];
            Vec3<T>::z_val_ = vec[2];
            w_val_ = vec[3];
        }

        virtual std::vector<T> asVector()
        {
            std::vector<T> vec{ Vec3<T>::x_val_, Vec3<T>::y_val_, Vec3<T>::z_val_, w_val_ };
            return vec;
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec4<T>& data)
        {
            os << "(" << data.x() << ", " << data.y() << ", " << data.z() << ", " << data.w() << ")";
            return os;
        }

        T w() const { return w_val_; };
        T a() const { return w(); }

    protected:
        T w_val_;
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
            position_.update(x, y, z);
        }

        void updatePosition(Vec3<T> pos)
        {
            position_.update(pos.x(), pos.y(), pos.z());
        }

        void updatePosition(std::vector<T> pos)
        {
            position_.update(pos[0], pos[1], pos[2]);
        } 

        void updateOrientation(T roll, T pitch, T yaw)
        {
            orientation_.update(roll, pitch, yaw);
        }

        void updateOrientation(Vec3<T> ori)
        {
            orientation_.update(ori.roll(), ori.pitch(), ori.yaw());
        }

        void updateOrientation(std::vector<T> ori)
        {
            orientation_.update(ori[0], ori[1], ori[2]);
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
            os << "Position" << pose.position_ << "; Orientation" << pose.orientation_;
            return os;
        }

        Vec3<T> position_;
        Vec3<T> orientation_;
    };

    template<typename T>
    using Vec3Array = std::vector<Vec3<T>>;

    template<typename T>
    using Vec4Array = std::vector<Vec4<T>>;

    template<typename T>
    using PoseArray = std::vector<Pose<T>>;

    static tf2::Quaternion toTf2Quaternion(Vec3<double> orientation_RPY)
    {
        tf2::Quaternion quat;
        quat.setRPY(orientation_RPY.roll(), orientation_RPY.pitch(), orientation_RPY.yaw());
        quat.normalize();

        return quat;
    }

    static Vec4<double> toQuaternion(Vec3<double> orientation_RPY)
    {
        tf2::Quaternion quat = toTf2Quaternion(orientation_RPY);
        return Vec4<double>(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    static Vec3<double> toRPY(Vec4<double> orientation_Quat)
    {
        tf2::Quaternion quat(orientation_Quat.x(), orientation_Quat.y(), 
                             orientation_Quat.z(), orientation_Quat.w());
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        return Vec3<double>(roll, pitch, yaw);
    }
}

}

#endif //DATALOADER_UTILS
