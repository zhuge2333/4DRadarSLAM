
#include <ros/ros.h>

#include <Eigen/Dense>

class CubicInterpolation {
    public:
    Eigen::MatrixXd q_via;
    Eigen::VectorXd t_via;
    CubicInterpolation(Eigen::MatrixXd q_via_, Eigen::VectorXd t_via_){
        /*
        :param: name: string
            name of objective
        :param: q_via: N x 3 array
            given q array
        :param: t_via: N x 1 array
            given t array
        */
        q_via = q_via_;
        t_via = t_via_;
        if(q_via.rows() != t_via.rows())
            ROS_ERROR("%s","The q_via and t_via must have a same length");
    }
    virtual ~CubicInterpolation() {}
    private:
    int cnt = 0;
    // void resetIntegration() {

    // }
    public:
    Eigen::Vector4d cubic(double q0, double q1, double v0, double v1, double t0, double t1){
        /*
        :param: q0: float
            the first data point
        :param: q1: float
            the second data point
        :param: v0: float
            the velocity of the first data point
        :param: v1: float
            the velocity of the second data point
        :param: t0: float
            the time of the first data point
        :param: t1: float
            the time of the second data point
        */
        if(abs(t0 - t1) < 1e-6)
            ROS_ERROR("%s","t0 and t1 must be different");

        double T = t1 - t0;
        double h = q1 - q0;

        double a0 = q0;
        double a1 = v0;
        double a2 = (3*h - (2*v0 + v1)*T) / (T*T);
        double a3 = (-2*h + (v0 + v1)*T) / (T*T*T);
        return Eigen::Vector4d(a0, a1, a2, a3);
    }

    double getPosition(double t){
        /*
        :param: t: float
            specified time
        :return: q: float
            output of the interpolation at time t
        */
        if(t < t_via(0) || t > t_via(t_via.size()-1))
            ROS_ERROR("%s","The specific time error, time ranges error");

        // j_array = np.where(self.t_via >= t); // find the index of t1
        int j;// = j_array[0][0];
        for (int i = 0; i < t_via.rows(); i++){
            if (t > t_via(i));
            else {j=i; break;}
        }
        int i;
        if (j == 0) {i = 0; j = 1;}
        else i = j-1;

        // get given position
        double q0 = q_via(i,0);
        double v0 = q_via(i,1);
        double t0 = t_via(i);

        double q1 = q_via(j,0);
        double v1 = q_via(j,1);
        double t1 = t_via(j);

        Eigen::Vector4d a = cubic(q0, q1, v0, v1, t0, t1);

        Eigen::Vector3d q;
        q(0) = a(0) + a(1)*(t - t0) + a(2)*pow(t-t0, 2) + a(3)*pow(t-t0, 3); // position
        // q(1) = a(1) + 2*a(2)*(t - t0) + 3*a(3)*pow(t-t0, 2); // velocity
        // q(2) = 2*a(2) + 6*a(3)*(t - t0); // acceleration

        return q(0);
    }
};

