#include "carla_l5player_hybridastar_planner/rs_curve.h"

Path ReedsSheppPath::rs_planning(double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size=0.2)
{
    std::vector<Path> paths;
    calc_allPath(paths, sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size);

    double minL = LONG_MAX;
    int minIndex = 0;

    int path_size = paths.size();
    if(path_size == 0){
        std::cout<<"path is not planned!"<<std::endl;
    }

    for(int i=0; i<path_size; ++i){
        if(paths[i].L <= minL){
            minL = paths[i].L;
            minIndex = i;
        }
    }

    return paths[minIndex];

}

void ReedsSheppPath::calc_allPath(std::vector<Path>& paths, double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size=0.2)
{
    generate_allPath(paths, sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size);

    for(auto& path : paths){
        std::vector<double> x, y, yaw;
        //unknown error measurement
        path.x.resize(10);
        path.y.resize(10);
        path.yaw.resize(10);
        path.lengths.resize(10);
        
        generate_local_course(x, y, yaw, path.directions, path, max_curvature, step_size*max_curvature);

        //convert global coordinate
        int path_size = x.size();
        path.x.resize(path_size);
        path.y.resize(path_size);
        path.yaw.resize(path_size);
        path.lengths.resize(path_size);
        for(int i = 0; i<path_size; ++i){
            path.x[i] = cos(-syaw)*x[i] + sin(-syaw)*y[i] + sx;
            path.y[i] = -sin(-syaw)*x[i] + cos(-syaw)*y[i] + sy;
            path.yaw[i] = arrangeAngle(yaw[i] + syaw);
            path.lengths[i] = path.lengths[i]/max_curvature;
        }
        path.L = path.L/max_curvature;
    }
}

void ReedsSheppPath::generate_allPath(std::vector<Path>& paths, double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size)
{
    double dx = gx - sx;
    double dy = gy - sy;
    double dyaw = gyaw - syaw;

    double c = cos(syaw);
    double s = sin(syaw);
    double x = (c*dx + s*dy)*max_curvature;
    double y = (-s*dx + c*dy)*max_curvature;

    SCS(x, y, dyaw, paths, step_size);
    CSC(x, y, dyaw, paths, step_size);
    CCC(x, y, dyaw, paths, step_size);
    CCCC(x, y, dyaw, paths, step_size);
    CCSC(x, y, dyaw, paths, step_size);
    CCSCC(x, y, dyaw, paths, step_size);
}

void ReedsSheppPath::SCS(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = SLS(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"S", "L", "S"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = SLS(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"S", "R", "S"};
        set_path(paths, lengths, ctypes, step_size);
    }
}

void ReedsSheppPath::CSC(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = LSL(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSL(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSL(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSL(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSR(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSR(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSR(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LSR(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
        std::cout<<paths.size()<<std::endl;
    }
}

void ReedsSheppPath::CCC(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = LRL(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    //backward
    double xb = x*cos(phi) + y*sin(phi);
    double yb = x*sin(phi) - y*cos(phi);

    detail = LRL(xb, yb, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, detail.t};
        std::vector<std::string> ctypes{"L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(-xb, yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, -detail.t};
        std::vector<std::string> ctypes{"L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(xb, -yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, detail.t};
        std::vector<std::string> ctypes{"R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRL(-xb, -yb, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, -detail.t};
        std::vector<std::string> ctypes{"R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }
}

void ReedsSheppPath::CCCC(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = LRLRn(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, -detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRn(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRn(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, -detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRn(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRp(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRp(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRp(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, detail.u, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRLRp(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, -detail.u, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }
}

void ReedsSheppPath::CCSC(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = LRSL(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    //backward
    double xb = x*cos(phi) + y*sin(phi);
    double yb = x*sin(phi) - y*cos(phi);

    detail = LRSL(xb, yb, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, -0.5*M_PI, detail.t};
        std::vector<std::string> ctypes{"L", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(-xb, yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, 0.5*M_PI, -detail.t};
        std::vector<std::string> ctypes{"L", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(xb, -yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, -0.5*M_PI, detail.t};
        std::vector<std::string> ctypes{"R", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSL(-xb, -yb, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, 0.5*M_PI, -detail.t};
        std::vector<std::string> ctypes{"R", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(xb, yb, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, -0.5*M_PI, detail.t};
        std::vector<std::string> ctypes{"R", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(-xb, yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, 0.5*M_PI, -detail.t};
        std::vector<std::string> ctypes{"R", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(xb, -yb, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.v, detail.u, -0.5*M_PI, detail.t};
        std::vector<std::string> ctypes{"L", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSR(-xb, -yb, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.v, -detail.u, 0.5*M_PI, -detail.t};
        std::vector<std::string> ctypes{"L", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

}

void ReedsSheppPath::CCSCC(double x, double y, double phi, std::vector<Path>& paths, double step_size)
{
    path_detail detail;

    detail = LRSLR(x, y, phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, -0.5*M_PI, detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSLR(-x, y, -phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, 0.5*M_PI, -detail.v};
        std::vector<std::string> ctypes{"L", "R", "S", "L", "R"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSLR(x, -y, -phi);
    if(detail.flag){
        std::vector<double> lengths{detail.t, -0.5*M_PI, detail.u, -0.5*M_PI, detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

    detail = LRSLR(-x, -y, phi);
    if(detail.flag){
        std::vector<double> lengths{-detail.t, 0.5*M_PI, -detail.u, 0.5*M_PI, -detail.v};
        std::vector<std::string> ctypes{"R", "L", "S", "R", "L"};
        set_path(paths, lengths, ctypes, step_size);
    }

}

ReedsSheppPath::path_detail ReedsSheppPath::LSL(double x, double y, double phi)
{
    path_detail detail;
    detail.u = polar_r(x - sin(phi), y - 1.0 + cos(phi));
    detail.t = polar_theta(x - sin(phi), y - 1.0 + cos(phi));

    if(detail.t >= 0.0){
        detail.v = arrangeAngle(phi - detail.t);
        if(detail.v >= 0.0){
            detail.flag = true;
            return detail;
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LSR(double x, double y, double phi)
{
    path_detail detail;

    double u1 = polar_r(x + sin(phi), y - 1.0 - cos(phi));
    double t1 = polar_theta(x + sin(phi), y - 1.0 - cos(phi));
    u1 *= u1;

    if(u1 >= 4.0){
        detail.u = sqrt(u1 - 4.0);
        double theta = atan2(2.0, detail.u);
        detail.t = arrangeAngle(t1 + theta);
        detail.v = arrangeAngle(detail.t - phi);

        if(detail.t>=0.0 && detail.v>=0.0){
            detail.flag = true;
            return detail;
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRL(double x, double y, double phi)
{
    path_detail detail;

    double u1 = polar_r(x - sin(phi), y - 1.0 + cos(phi));
    double t1 = polar_theta(x - sin(phi), y - 1.0 + cos(phi));

    if(u1 <= 4.0){
        detail.u = -2.0 * asin(0.25*u1);
        detail.t = arrangeAngle(t1 + 0.5*detail.u + M_PI);
        detail.v = arrangeAngle(phi - detail.t + detail.u);

        if(detail.t>=0.0 && detail.u<=0.0){
            detail.flag = true;
            return detail;
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::SLS(double x, double y, double phi)
{
    path_detail detail;

    phi = arrangeAngle(phi);

    if(y > 0.0 && (0.0 < phi && phi < M_PI*0.99)){
        double xd = -y/tan(phi) + x;
        detail.t = xd - tan(phi/2.0);
        detail.u = phi;
        detail.v = sqrt((x-xd)*(x-xd) + y*y) - tan(phi/2.0);
        detail.flag = true;

        return detail;
    }else if(y < 0.0 && (0.0 < phi && phi < M_PI*0.99)){
        double xd = -y/tan(phi) + x;
        detail.t = xd - tan(phi/2.0);
        detail.u = phi;
        detail.v = -sqrt((x-xd)*(x-xd) + y*y) - tan(phi/2.0);
        detail.flag = true;

        return detail;
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRLRn(double x, double y, double phi)
{
    path_detail detail;

    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = 0.25*(2.0 + sqrt(xi*xi + eta*eta));

    if(rho <=1.0){
        detail.u = acos(rho);
        calc_tauOmega(detail.t, detail.v, detail.u, -detail.u, xi, eta, phi);
        if(detail.t >= 0.0 && detail.v <= 0.0){
            detail.flag = true;
            return detail;
        }

    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRLRp(double x, double y, double phi)
{
    path_detail detail;

    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = (20.0 - xi*xi - eta*eta)/16.0;

    if(0.0 <= rho && rho <=1.0){
        detail.u = -acos(rho);
        if(detail.u >= -0.5*M_PI){
            calc_tauOmega(detail.t, detail.v, detail.u, detail.u, xi, eta, phi);
            if(detail.t >= 0.0 && detail.v >= 0.0){
                detail.flag = true;
                return detail;
            }
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRSR(double x, double y, double phi)
{
    path_detail detail;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = polar_r(-eta, xi);
    double theta = polar_theta(-eta, xi);

    if(rho >= 2.0){
        detail.t = theta;
        detail.u = 2.0 - rho;
        detail.v = arrangeAngle(detail.t + 0.5*M_PI - phi);

        if((detail.t >= 0.0 && detail.u <= 0.0) && detail.v <= 0.0){
            detail.flag = true;
            return detail;
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRSL(double x, double y, double phi)
{
    path_detail detail;
    double xi = x - sin(phi);
    double eta = y - 1.0 + cos(phi);
    double rho = polar_r(xi, eta);
    double theta = polar_theta(xi, eta);

    if(rho >= 2.0){
        double r = sqrt(rho*rho - 4.0);
        detail.u = 2.0 - r;
        detail.t = arrangeAngle(theta + atan2(r, -2.0));
        detail.v = arrangeAngle(phi - 0.5*M_PI - detail.t);

        if((detail.t >= 0.0 && detail.u <= 0.0) && detail.v <= 0.0){
            detail.flag = true;
            return detail;
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

ReedsSheppPath::path_detail ReedsSheppPath::LRSLR(double x, double y, double phi)
{
    path_detail detail;
    double xi = x + sin(phi);
    double eta = y - 1.0 - cos(phi);
    double rho = polar_r(xi, eta);
    double theta = polar_theta(xi, eta);

    if(rho >= 2.0){
        detail.u = 4.0 - sqrt(rho*rho - 4.0);
        if(detail.u <= 0.0){
            detail.t = arrangeAngle(atan2((4.0-detail.u)*xi - 2.0*eta, -2.0*xi + (detail.u - 4.0)*eta));
            detail.v = arrangeAngle(detail.t - phi);

            if(detail.t>=0.0 && detail.v >= 0.0){
                detail.flag = true;
                return detail;
            }
        }
    }

    detail.flag = false;
    detail.t = 0.0;
    detail.u = 0.0;
    detail.v = 0.0;

    return detail;
}

void ReedsSheppPath::set_path(std::vector<Path>& paths, const std::vector<double>& lengths, const std::vector<std::string>& ctypes, double step_size)
{
    Path path;
    path.ctypes = ctypes;
    path.lengths = lengths;
    for(const auto& length : lengths){
        path.L += abs(length);
    }

    //check same path exist
    for(const auto& path_i : paths){
        bool type_is_same = (path_i.ctypes == path.ctypes);
        double l = 0;
        for(const auto& length : path_i.lengths){
            l += abs(length);
        }
        bool length_is_close = (l - path.L) <= step_size;
        if(type_is_same && length_is_close){
            return; //same path found, so do not insert path
        }

    }
    //check path is long enough
    if(path.L <= step_size){
        return; //too short, so do not insert path
    }

    paths.push_back(path);
}

void ReedsSheppPath::generate_local_course(std::vector<double>& px, std::vector<double>& py, std::vector<double>& pyaw, std::vector<bool> directions, const Path& path, double max_curvature, double step_size)
{
    int point_num = (int)path.L/step_size + path.lengths.size() + 3;

    px.resize(point_num);
    py.resize(point_num);
    pyaw.resize(point_num);
    directions.resize(point_num);
    int ind = 1;
    double d;

    if(path.lengths[0] > 0.0){
        directions[0]  = true;
        d = step_size;
    }else{
        directions[0]  = false;
        d = -step_size;
    }

    double pd = d;
    double ll = 0.0;

    const int mode_size = path.ctypes.size();
    for(int i=0; i<mode_size; ++i){
        if(path.lengths[i] > 0.0){
            d = step_size;
        }else{
            d = -step_size;
        }

        double ox = px[ind];
        double oy = py[ind];
        double oyaw = pyaw[ind];

        ind--;
        if(i >= 1 && (path.lengths[i-1]*path.lengths[i] > 0.0)){
            pd = -d -ll;
        }else{
            pd = d - ll;
        }

        while(abs(pd) <= abs(path.lengths[i]))
        {
            ind++;
            interpolate(px, py, pyaw, directions, ind, pd, path.ctypes[i], max_curvature, ox, oy, oyaw);
            pd += d;
        }

        ll = path.lengths[i] - pd - d; //calc remain length

        ind++;
        interpolate(px, py, pyaw, directions, ind, path.lengths[i], path.ctypes[i], max_curvature, ox, oy, oyaw);
    }

    //remove unused data
    while(px.back() == 0.0){
        px.pop_back();
        py.pop_back();
        pyaw.pop_back();
        directions.pop_back();
    }

}

void ReedsSheppPath::interpolate(std::vector<double>& px, std::vector<double>& py, std::vector<double>& pyaw, std::vector<bool>& directions,
                     int ind, double length, std::string mode, double maxc, double ox, double oy, double oyaw)
{
    if(mode == "S"){
        px[ind] = ox + length/maxc*cos(oyaw);
        py[ind] = oy + length/maxc*sin(oyaw);
        pyaw[ind] = oyaw;
    }else{
        double ldx = sin(length)/maxc;
        double ldy;
        if(mode == "L"){
            ldy = (1.0 - cos(length))/maxc;
        }else if(mode == "R"){
            ldy = (1.0 - cos(length))/(-maxc);
        }

        double gdx = cos(-oyaw)*ldx + sin(-oyaw)*ldy;
        double gdy = -sin(-oyaw)*ldx + cos(-oyaw)*ldy;
        px[ind] = ox + gdx;
        py[ind] = oy + gdy;
    }

    if(mode == "L"){
        pyaw[ind] = oyaw + length;
    }else if(mode == "R"){
        pyaw[ind] = oyaw - length;
    }

    if(length > 0.0){
        directions[ind] = true;
    }else{
        directions[ind] = false;
    }
}