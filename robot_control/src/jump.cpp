#include "jump.h"

Jump::Jump(double time_step){
    this->dt = time_step;
    file.open("/home/kassra/Thesis/choreonoid_ws/src/thesis/log/jump.csv");
}

Jump::~Jump(){
    file.close();
}

double* Jump::generateTraj(double l_0, double l_d_0, double l_dd_0, 
                double l_to, double t_0, double t_to, double t_recover, double zd, int &len){
    // Define Constraints on Trajectories
    double l_dd_to = -9.81;
    double l_td = l_to;
    double l_dd_td = -9.81;
    double l_d_to = sqrt(2 * 9.81 * (zd - l_to));
    double l_d_td = -l_d_to;
    double t_td = t_to + 2 * l_d_to / 9.81;
    t_recover += t_td;

    double* res = new double[int(t_recover / this->dt)];
    double coefs[6];

    this->fitPoly5(coefs, l_0, l_d_0, l_dd_0, l_to, l_d_to, l_dd_to, t_to - t_0);
    int i = 0;
    for (double time = t_0; time < t_to; time += this->dt){
        res[i] = coefs[0] + coefs[1] * (time - t_0) + coefs[2] * pow((time - t_0), 2) + coefs[3] * pow((time - t_0), 3) + 
                + coefs[4] * pow((time - t_0), 4) + coefs[5] * pow((time - t_0), 5);
        file << time << ", " << res[i];
        res[i] = this->CoM2Pelvis(res[i]);
        file << ", " << res[i] << endl;
        i++;
    }

    this->fitPoly5(coefs, l_to, l_d_to, l_dd_to, l_td, l_d_td, l_dd_td, t_td-t_to);
    for (double time = t_to; time < t_td; time += this->dt){
        res[i] = coefs[0] + coefs[1] * (time - t_to) + coefs[2] * pow((time - t_to), 2) + coefs[3] * pow((time - t_to), 3) + 
                + coefs[4] * pow((time - t_to), 4) + coefs[5] * pow((time - t_to), 5);
        file << time << ", " << res[i];
        //res[i] = this->CoM2Pelvis(res[i]);
        res[i] = res[i-1];
        file << ", " << res[i] << endl;
        i++;
    }

    this->fitPoly5(coefs, l_td, l_d_td, l_dd_td, l_0, l_d_0, l_dd_0, t_recover - t_td);
    for (double time = t_td; time < t_recover; time += this->dt){
        res[i] = coefs[0] + coefs[1] * (time - t_td) + coefs[2] * pow((time - t_td), 2) + coefs[3] * pow((time - t_td), 3) + 
                + coefs[4] * pow((time - t_td), 4) + coefs[5] * pow((time - t_td), 5);
        file << time << ", " << res[i];
        res[i] = this->CoM2Pelvis(res[i]);
        file << ", " << res[i] << endl;
        i++;
    }

    len = i;
    return res;

}

bool Jump::fitPoly5(double (&res)[6], double init_x, double init_xd, 
                double init_xdd,double end_x, double end_xd, double end_xdd, double t){
    res[0] = init_x;
    res[1] = init_xd;
    res[2] = 0.5 * init_xdd;
    res[3] = (end_xdd * pow(t, 2) - 3 * pow(t, 2) * init_xdd - 8 * end_xd * t - 12 * init_xd * t + 20 * end_x - 20 * init_x) / pow(t, 3) / 2;
    res[4] = -(2 * end_xdd * pow(t, 2) - 3 * pow(t, 2) * init_xdd - 14 * end_xd * t - 16 * init_xd * t + 30 * end_x - 30 * init_x) / pow(t, 4) / 2;
    res[5] = (end_xdd * pow(t, 2) - pow(t, 2) * init_xdd - 6 * end_xd * t - 6 * init_xd * t + 12 * end_x - 12 * init_x) / pow(t, 5) / 2;

    return true;
}

double Jump::CoM2Pelvis(double com){
    return 1.19214696 * com + -0.00361358;
}
