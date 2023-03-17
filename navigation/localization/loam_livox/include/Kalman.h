#include "Eigen/Dense"

class KalmanFilter{
public:
    KalmanFilter(){
        is_initialized_ = false;
    };

    // ~KalmanFilter();
    
    Eigen::VectorXd GetX(){
        return x_;
    }

    bool IsInitialized(){
        return is_initialized_;
    }

    void Initialization(Eigen::VectorXd x_in){
        x_ = x_in;
        is_initialized_ = true;
    }

    void SetF(Eigen::MatrixXd F_in){
        F_ = F_in;
    }

    void SetP(Eigen::MatrixXd P_in){
        P_ = P_in;
    }
    
    void SetQ(Eigen::MatrixXd Q_in){
        Q_ = Q_in;
    }

    void SetH(Eigen::MatrixXd H_in){
        H_ = H_in;
    }

    void SetR(Eigen::MatrixXd R_in){
        R_ = R_in;
    }

    void Prediction(){
        x_ = F_*x_;
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_ *P_*Ft + Q_;
    }

    void MeasurementUpdate(const Eigen::VectorXd &z){
        Eigen::VectorXd y = z - H_*x_;
        Eigen::MatrixXd S = H_*P_*H_.transpose()+R_;
        Eigen::MatrixXd K = P_*H_.transpose()*S.inverse();
        x_ = x_+(K*y);
        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I-K*H_)*P_;
    }
private:
    //flag of initialization
    bool is_initialized_;

    //state vector
    Eigen::VectorXd x_;

    //state transition matrix
    Eigen::MatrixXd F_;

    //state covariance matrix
    Eigen::MatrixXd P_;

    //process covariance matrix
    Eigen::MatrixXd Q_;

    Eigen::MatrixXd H_;

    Eigen::MatrixXd R_;

};
