#include <cmath>

#define PI std::acos(-1)
#define DEG2RAD PI/180
#define RAD2DEG 180/PI

namespace MyMathFun{
	void quaternion_2_euler(double quat[4], double angle[3]){
		angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
		angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
		angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
	}
	
	void Euler_2_Dcm(double euler[3], double dcm[3][3]){
		double phi = euler[0], theta = euler[1], psi = euler[2];
		double sinPhi = sin(phi), cosPhi = cos(phi);
		double sinThe = sin(theta), cosThe = cos(theta);
		double sinPsi = sin(psi), cosPsi = cos(psi);
		dcm[0][0] = cosThe * cosPsi;
		dcm[0][1] = cosThe * sinPsi; 
		dcm[0][2] = -sinThe; 

		dcm[1][0] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
		dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
		dcm[1][2] = sinPhi * cosThe;

		dcm[2][0] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;
		dcm[2][1] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;
		dcm[2][2] = cosPhi * cosThe;
	}

    double LimitValue(double x, double sat){
        return std::min(std::abs(sat), std::max(-std::abs(sat), x));
    }

}
	