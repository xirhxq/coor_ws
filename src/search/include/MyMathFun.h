#include <cmath>
#include <vector>

#define PI std::acos(-1)
#define DEG2RAD PI/180
#define RAD2DEG 180/PI

namespace MyMathFun{
	struct DATA_STAT{
		int cnt;
		double mean, std, rms;
		DATA_STAT(){
		cnt = 0;
		mean = std = rms = 0.0;
		}
		void new_data(double x){
			cnt++;
			std = pow(std, 2) / cnt * (cnt - 1) + (x - mean) * (x - mean) / cnt * (cnt - 1) / cnt;
			std = pow(std, 0.5);
			mean = mean / cnt * (cnt - 1) + x / cnt;
			rms = pow(rms, 2) / cnt * (cnt - 1) + x * x / cnt;
			rms = pow(rms, 0.5);
		}
	};

	struct Filter{
		std::vector<double> v;
		size_t size;

		Filter(size_t sz_ = 3){
			size = sz_;
			while (!v.empty()) v.erase(v.begin());
		}

		void new_data(double nd_){
			v.push_back(nd_);
			// printf("push %lf size %ld\n", nd_, v.size());
			// output();
			while (v.size() > size) v.erase(v.begin());
			// printf("now size %ld\n", v.size());
			// output();
		}

		double result(){
			std::vector<double> tmp = v;
			std::sort(tmp.begin(), tmp.end());
			return (tmp[(tmp.size() - 1) / 2] + tmp[tmp.size() / 2]) / 2;
		}

		void output(){
			printf("Now Filter Contains:");
			for (auto i: v) printf("\t%lf", i);
			printf("\n");
		}
	};

	template<typename T>
	struct XYZ_Filter{
		Filter x, y, z;
		XYZ_Filter(int sz_ = 3): x(sz_), y(sz_), z(sz_){
			
		}

		void new_data(T nd_){
			// printf("push x\n");
			x.new_data(nd_.x);
			// printf("push y\n");
			y.new_data(nd_.y);
			// printf("push z\n");
			z.new_data(nd_.z);
		}

		T result(){
			T res;
			res.x = x.result();
			res.y = y.result();
			res.z = z.result();
			return res;
		}

		void output(){
			x.output();
			y.output();
			z.output();
		}
	};

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
    
    //solve target angle from zc
	void angle_transf(double euler[3], double bias, double pixel[2],double q[3])
	{
		//q_alpha: pitch->pixel[1]
		//q_beta: 
		pixel[0] = -pixel[0]; 
		double theta = euler[1]+bias;
		double M1 = -cos(theta)*sin(euler[2])*cos(pixel[1])*cos(pixel[0])+(sin(theta)*sin(euler[2])*cos(euler[0])+cos(euler[2])*sin(euler[0]))*sin(pixel[1])-
				(-sin(theta)*sin(euler[2])*sin(euler[0])+cos(euler[2])*cos(euler[0]))*cos(pixel[1])*sin(pixel[0]);
		double N1 = cos(theta)*cos(euler[2])*cos(pixel[1])*cos(pixel[0])+(-sin(theta)*cos(euler[2])*cos(euler[0])+sin(euler[2])*sin(euler[0]))*sin(pixel[1])-
				(sin(theta)*cos(euler[2])*sin(euler[0])+sin(euler[2])*cos(euler[0]))*cos(pixel[1])*sin(pixel[0]);
				
		q[1] = asin(sin(theta)*cos(pixel[1])*cos(pixel[0])+cos(theta)*cos(euler[0])*sin(pixel[1])+cos(theta)*sin(euler[0])*cos(pixel[1])*sin(pixel[0]));
		q[2] = atan2(-M1,N1);
	}

}
	
