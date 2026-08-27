#include <cmath>
#include <iostream>
#include <iomanip>

int main()
{
    const int N = 1000;

    for (double L : {40.0, 60.0})
    {
        std::cout << "\n=== L=" << L << " ===\n";

        for (int i = 1; i <= 200; ++i)
        {
            double omega = i * 0.005;
            double xL = 0, yL = 0;
            double ds = L / N;
            for (int j = 0; j < N; ++j)
            {
                double s = (j + 0.5) * ds;
                double theta = omega * s * (1.0 - s / L);
                xL += std::cos(theta) * ds;
                yL += std::sin(theta) * ds;
            }
            if (std::abs(yL) < 2.0)
            {
                std::cout << "w=" << std::fixed << std::setprecision(3) << omega
                    << "  x=" << std::setprecision(4) << xL
                    << "  y=" << yL
                    << "  gap=" << (xL + 30.0) << "\n";
            }
        }

        double lo = 0.2, hi = 0.6;
        for (int iter = 0; iter < 80; ++iter)
        {
            double mid = (lo + hi) * 0.5;
            double yMid = 0;
            double ds = L / N;
            for (int j = 0; j < N; ++j)
            {
                double s = (j + 0.5) * ds;
                double theta = mid * s * (1.0 - s / L);
                yMid += std::sin(theta) * ds;
            }
            if (yMid > 0) lo = mid; else hi = mid;
        }
        double omegaOpt = (lo + hi) * 0.5;

        double xOpt = 0, yOpt = 0;
        double ds = L / N;
        for (int j = 0; j < N; ++j)
        {
            double s = (j + 0.5) * ds;
            double theta = omegaOpt * s * (1.0 - s / L);
            xOpt += std::cos(theta) * ds;
            yOpt += std::sin(theta) * ds;
        }

        std::cout << "OPTIMAL w=" << std::setprecision(6) << omegaOpt
            << "  wL=" << (omegaOpt * L)
            << "  x(L)=" << std::setprecision(4) << xOpt
            << "  y(L)=" << std::setprecision(6) << yOpt
            << "  gap=" << std::setprecision(4) << (xOpt + 30.0) << "\n";
    }
    return 0;
}
