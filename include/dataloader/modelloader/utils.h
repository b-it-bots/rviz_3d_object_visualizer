#ifndef DATALOADER_UTILS
#define DATALOADER_UTILS

#include<iostream>
#include<vector>

namespace RVizDataLoader 
{

namespace Utils
{
    template<typename T>
    class Vec3
    {
    public:
        Vec3(): x_val(T{}), y_val(T{}), z_val(T{}) {}

        void update(T x, T y, T z)
        {
            x_val = x;
            y_val = y;
            z_val = z;
        }

        void update(std::vector<T> vec)
        {
            x_val = vec[0];
            y_val = vec[1];
            z_val = vec[2];
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec3<T>& data)
        {
            std::cout << "(" << data.x << ", " << data.y << ", " << data.z << ")";
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
            std::cout << "Position" << pose.position << "; Orientation" << pose.orientation;
        }

        Vec3<T> position;
        Vec3<T> orientation;
    };
}

}

#endif //DATALOADER_UTILS
