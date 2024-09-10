#include "Trigonometric.h"

double nav_Pow(double x, int n)                // 要计算的值(x)和要计算的次方(n)，传出值：x的n次方
{
    double res = 1.0;
    for(int i = n; i != 0; i /= 2){
        if(i % 2 != 0){
            res *= x;
        }
        x *= x;
    }
    return  n < 0 ? 1 / res : res;
}

double sin_180(double direction)
{
    if(direction > 180)
        return -1;                                  //  移植的时候改为 rt_error

    double angle[13] = {45.0, 26.565051177077986, 14.036243467926479, 7.125016348901799,
                        3.5763343749973515, 1.7899106082460694, 0.8951737102110744,
                        0.4476141708605531, 0.22381050036853808, 0.1119056770662069,
                        0.05595289189380367, 0.02797645261700368, 0.013988227142265015};		// 12 times
    double x=1;
    double y=0;
    double k=0.60723;  // 旋转矩阵比例因子
    double x_new, y_new;

    for(int i = 0; i < 13; i++)
    {
        angle[i] = (angle[i] / 45) * nav_Pow(2, 20);
    }

    direction = (direction / 45) * nav_Pow(2, 20);

    for(int i = 0; i < 13; i++)
    {
        if(direction > 0)
        {
            x_new = (x - y * (1 / nav_Pow(2, i)));
            y_new = (y + x * (1 / nav_Pow(2, i)));
            x = x_new;
            y = y_new;
            direction -= angle[i];
        }
        else{
            x_new = (x + y * (1 / nav_Pow(2, i)));
            y_new = (y - x * (1 / nav_Pow(2, i)));
            x = x_new;
            y = y_new;
            direction += angle[i];
        }
    }

    return y * k;
}

double sin_360(double direction)
{
    if(direction < 180)
        return -1;                                  //  移植的时候改为 rt_error
    return -sin_180(direction - 180);
}

double nav_sin(double direction)
{
    if(direction <= 180)
        return sin_180(direction);
    if(direction > 180)
        return sin_360(direction);
	return 0;
}

double cos_180(double direction)
{
    if(direction <= 90)
        return sin_180(90 - direction);
    else if(direction > 90 && direction <= 180)
        return -sin_180(direction - 90);
    return -1;                                      //  移植的时候改为 rt_error
}

double cos_360(double direction)
{
    if(direction > 180)
        return cos_180(360 - direction);
    else
        return -1;                                  //  移植的时候改为 rt_error
}

double nav_cos(double direction)
{
    if(direction <= 180)
        return cos_180(direction);
    if(direction > 180)
        return cos_360(direction);
	return 0;
}

double nav_tan(double direction)
{
    return nav_sin(direction) / nav_cos(direction);
}