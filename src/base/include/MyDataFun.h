#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cstring>

typedef geometry_msgs::msg::Point Point;


namespace MyDataFun{
    template<typename T1, typename T2>
    void set_value(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
    }

    template<typename T>
    void set_value(T &a, double b[]){
        a.x = b[0];
        a.y = b[1];
        a.z = b[2];
    }

    template<typename T1, typename T2>
    void set_value_quaternion(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
        a.w = b.w;
    }

    Point new_point(double x, double y, double z){
        Point res;
        double p[3] = {x, y, z};
        set_value(res, p);
        return res;
    }

    Point new_point(double p[]){
        Point res;
        set_value(res, p);
        return res;
    }

    template<typename T1, typename T2>
    double dis(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
    }

    template<typename T1, typename T2>
    double dissq(T1 a, T2 b){
        return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
    }

    template<typename T1, typename T2>
    double dis_2d(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    template<typename T1, typename T2>
    double angle_2d(T1 from, T2 to){
        return atan2(to.y - from.y, to.x - from.x);
    }

    template<typename T1, typename T2>
    T1 minus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        return res;
    }    

    template<typename T1, typename T2>
    T1 plus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        res.z = a.z + b.z;
        return res;
    }

    template<typename T>
    std::string output_str(T a){
        char s[50];
        sprintf(s, "(%.2lf, %.2lf, %.2lf)", a.x, a.y, a.z);
        std::string res(s);
        return res;
    }

}

    