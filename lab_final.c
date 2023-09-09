#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -25.62*PI/180; //-0.37;
float offset_Enc3_rad = 12.81*PI/180; //0.27;


// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];


long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float theta_dh1 = 0;
float theta_dh2 = 0;
float theta_dh3 = 0;



float theta_m1_inv = 0;
float theta_m2_inv = 0;
float theta_m3_inv = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float tau1_test = 0.0;
float tau2_test = 0.0;
float tau3_test = 0.0;

float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;
float Omega1_raw = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;
float Omega2_raw = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float Omega3_raw = 0;


float tau_cmd1 = 0;
float tau_cmd2 = 0;
float tau_cmd3 = 0;

float fric_factor = 0.8;
float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;

float visc_pos1 = 0.265;
float visc_neg1 = 0.29;
float minVel1 = 0.1;
float minSlope1 = 3.6;
float coul_pos1 = 0.38;
float coul_neg1 = -0.325;

float visc_pos2 = 0.25;
float visc_neg2 = 0.287;
float minVel2 = 0.05;
float minSlope2 = 3.6;
float coul_pos2 = 0.4759;
float coul_neg2 = -0.5031;

float visc_pos3 = 0.1922;
float visc_neg3 = 0.2132;
float minVel3 = 0.05;
float minSlope3 = 3.6;
float coul_pos3 = 0.5339;
float coul_neg3 = -0.5190;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

float Kpx = 0.75;
float Kdx = 0.025;
float u_pdx_N = 0;
float u_pdx = 0;

float Kpy = 0.13;
float Kdy = 0.013;
float u_pdy_N = 0;
float u_pdy = 0;

float Kpz = 0.5;
float Kdz = 0.025;
float u_pdz_N = 0;
float u_pdz = 0;

float u_pd1 = 0;
float u_pd2 = 0;
float u_pd3 = 0;

float xd = 13.435028842544401;
float xd_dot = 0;
float xd_N = 0;
float xd_dot_N = 0;

float yd = 13.435028842544401;
float yd_dot = 0;
float yd_N = 0;
float yd_dot_N = 0;

float zd = 10;
float zd_dot = 0;
float zd_N = 0;
float zd_dot_N = 0;

float px = 0;
float px_N = 0;
float py = 0;
float py_N = 0;
float pz = 0;
float pz_N = 0;

float px_old = 0;
float x_dot = 0;
float x_dot_N = 0;
float x_dot_raw = 0;
float x_dot_old1 = 0;
float x_dot_old2 = 0;


float py_old = 0;
float y_dot = 0;
float y_dot_N = 0;
float y_dot_raw = 0;
float y_dot_old1 = 0;
float y_dot_old2 = 0;


float pz_old = 0;
float z_dot = 0;
float z_dot_N = 0;
float z_dot_raw = 0;
float z_dot_old1 = 0;
float z_dot_old2 = 0;

float Fzcmd = 0;
float Kt = 6;

float err_x = 0;
float err_x_N = 0;
float err_x_dot = 0;
float err_x_dot_N;
float err_y = 0;
float err_y_N = 0;
float err_y_dot = 0;
float err_y_dot_N;
float err_z = 0;
float err_z_N = 0;
float err_z_dot = 0;
float err_z_dot_N = 0;

float u_app1 = 0;
float u_app2 = 0;
float u_app3 = 0;

float desired_vel = 0.002; // inches per millisecond
float pt1_x = 7;
float pt1_y = 7;
float pt1_z = 10;

float pt2_x = 13;
float pt2_y = 13;
float pt2_z = 10;

float pt3_x = 7;
float pt3_y = 13;
float pt3_z = 10;

float grav_coeff2 = 0;
float grav_coeff3 = 0;

typedef struct point_tag {
    float x;
    float y;
    float z;
    float thz;
    int mode;
    float speed;

} point;

#define XYZSTIFF 1
#define ZSTIFF 2
#define XZSTIFF 3
#define XYSTIFF_ZZERO 4


int pointIndex = 0;
float t_start = 0;
float Fz_egg = -2.3;
float t_egg = 2.8;

#define NUM_POINTS 19
point point_array[NUM_POINTS] = {{5.42, 0, 16.79, 0, XYZSTIFF, 10}, // Point 0: Starting Point
                                 {0.7, 13.75, 10, 0, XYZSTIFF, 8}, // Point 1: High Above Hole
                                 {0.7, 13.75, 7.6, 0, ZSTIFF, 6}, // Point 2: Right Above Hole
                                 {0.7, 13.75, 5, 0, XYZSTIFF, 8}, // Point 3: Inside Hole
                                 {0.7, 13.75, 12.5, 0, XYZSTIFF, 10}, // Point 4: Out of Hole
                                 {8.75, 4.26, 12.8, 0, XYZSTIFF, 20}, // Point 5: Avoiding Box
                                 {14.5, 4.62, 12.8, 0, XYZSTIFF, 14}, // Point 6: Above Zigzag Entry Point
                                 {14.5, 4.62, 8.25, -0.925548102332593, XYZSTIFF, 15}, // Point 7: Zigzag Entry Point
                                 {15.87, 2.98, 8.25, -1.551190995937692, XZSTIFF, 2.5}, // Point 8: Zigzag - First Corner
                                 {15.88, 2.47, 8.25, 75*PI/180, XYZSTIFF, 17}, // Point 9: Zigzag - Rounding First Corner
                                 {12.95, 2.64, 8.25, -2.231839495645582, XYZSTIFF, 2.5}, // Point 10: Zigzag - Second Corner
                                 {12.46, 2.01, 8.25, -0.925548102332593, XYZSTIFF, 20}, // Point 11: Zigzag - Rounding Second Corner
                                 {15.08, -1.10, 8.21, 0, XYZSTIFF, 20}, // Point 12: Zigzag - Exit
                                 {15.08, -1.10, 12.5, 0, XYZSTIFF, 15}, // Point 13: Move Above Zigzag
                                 {9.18, 7.59, 12.5, 0, XYZSTIFF, 1}, // Point 14: Move Above Egg
                                 {9.18, 7.59, 11.8, 0, XYSTIFF_ZZERO, 1}, // Point 15: Touch and Push Egg
                                 {9.18, 7.59, 11.8, 0, XYZSTIFF, 1}, // Point 16: Move from Egg
                                 {9.18, 7.59, 12.5, 0, XYZSTIFF, 10}, // Point 17: Above Egg,
                                 {10, 0, 20, 0, XYZSTIFF, 6} // Point 18: Above Egg
                                };
point fromPt = {5.42, 0, 16.79, 0, XYZSTIFF, 0.002, 0};
point toPt = {0.79, 13.71, 13, 0, XYZSTIFF, 0.002, 0};
float t_total = 0;


void calcLine(float* x_desired, float* y_desired, float* z_desired, float* x_desired_d, float* y_desired_d, float* z_desired_d, long time_ms, float velocity) {

    float time = time_ms*0.001;


    if ((time - t_start + 0.0005) >= t_total) {
        int fromInd = pointIndex;
        int toInd = pointIndex + 1;

        if (toInd == NUM_POINTS) {
            toInd = 0;
            pointIndex = 0;
        } else{
            pointIndex++;
        }
        fromPt = point_array[fromInd];
        toPt = point_array[toInd];
        if (fromPt.mode == XYZSTIFF) {
            Kpx = 0.5;
            Kpy = 0.5;
            Kpz = 0.5;
        } else if (fromPt.mode == ZSTIFF) {
            Kpx = 0.1;
            Kpy = 0.1;
            Kpz = 0.5;
        } else if (fromPt.mode == XZSTIFF) {
            Kpx = 0.5;
            Kpy = 0.1;
            Kpz = 0.5;
        } else if (fromPt.mode == XYSTIFF_ZZERO) {
            Kpx = 0.5;
            Kpy = 0.5;
            Kpz = 0.05;
        }
        t_start = time;
        if (fromPt.mode == XYSTIFF_ZZERO) {
            t_total = t_egg;
            Fzcmd = -Fz_egg;
        } else {
            t_total = sqrt((fromPt.x - toPt.x)*(fromPt.x - toPt.x) + (fromPt.y - toPt.y)*(fromPt.y - toPt.y) + (fromPt.z - toPt.z)*(fromPt.z - toPt.z))/fromPt.speed;
            Fzcmd = 0;
        }
        thetaz = fromPt.thz;

    }

    *x_desired = (toPt.x - fromPt.x)*(time - t_start)/(t_total) + fromPt.x;
    *y_desired = (toPt.y - fromPt.y)*(time - t_start)/(t_total) + fromPt.y;
    *z_desired = (toPt.z - fromPt.z)*(time - t_start)/(t_total) + fromPt.z;

    *x_desired_d = (toPt.x - fromPt.x)/(t_total);
    *y_desired_d = (toPt.y - fromPt.y)/(t_total);
    *z_desired_d = (toPt.z - fromPt.z)/(t_total);

    
}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    Omega1_raw = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2_raw = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3_raw = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -10*sinq1*(cosq3 + sinq2);
    JT_12 = 10*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10*cosq1*(cosq2 - sinq3);
    JT_22 = 10*sinq1*(cosq2 - sinq3);
    JT_23 = -10*(cosq3 + sinq2);
    JT_31 = -10*cosq1*sinq3;
    JT_32 = -10*sinq1*sinq3;
    JT_33 = -10*cosq3;

    // Rotation Matrix R^W_N
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    // Task Space PD Control
    px = 10*cosq1*(cosq3 + sinq2);
    py = 10*sinq1*(cosq3 + sinq2);
    pz = 10*(1 + cosq2 - sinq3);

    x_dot_raw = (px - px_old)/0.001;
    x_dot = (x_dot_raw + x_dot_old1 + x_dot_old2)/3.0;
    px_old = px;
    
    x_dot_old2 = x_dot_old1;
    x_dot_old1 = x_dot;

    y_dot_raw = (py - py_old)/0.001;
    y_dot = (y_dot_raw + y_dot_old1 + y_dot_old2)/3.0;
    py_old = py;
    
    y_dot_old2 = y_dot_old1;
    y_dot_old1 = y_dot;

    z_dot_raw = (pz - pz_old)/0.001;
    z_dot = (z_dot_raw + z_dot_old1 + z_dot_old2)/3.0;
    pz_old = pz;
    
    z_dot_old2 = z_dot_old1;
    z_dot_old1 = z_dot;

    calcLine(&xd, &yd, &zd, &xd_dot, &yd_dot, &zd_dot, mycount, desired_vel);

    err_x = xd - px;
    err_y = yd - py;
    err_z = zd - pz;

    err_x_dot = xd_dot - x_dot;
    err_y_dot = yd_dot - y_dot;
    err_z_dot = zd_dot - z_dot;

    err_x_N = RT11*err_x + RT12*err_y + RT13*err_z;
    err_y_N = RT21*err_x + RT22*err_y + RT23*err_z;
    err_z_N = RT31*err_x + RT32*err_y + RT33*err_z;

    err_x_dot_N = RT11*err_x_dot + RT12*err_y_dot + RT13*err_z_dot;
    err_y_dot_N = RT21*err_x_dot + RT22*err_y_dot + RT23*err_z_dot;
    err_z_dot_N = RT31*err_x_dot + RT32*err_y_dot + RT33*err_z_dot;

    u_pdx_N = Kpx*err_x_N + Kdx*err_x_dot_N;
    u_pdy_N = Kpy*err_y_N + Kdy*err_y_dot_N;
    u_pdz_N = Kpz*err_z_N + Kdz*err_z_dot_N;

    u_pdx = R11*u_pdx_N + R12*u_pdy_N + R13*u_pdz_N;
    u_pdy = R21*u_pdx_N + R22*u_pdy_N + R23*u_pdz_N;
    u_pdz = R31*u_pdx_N + R32*u_pdy_N + R33*u_pdz_N;

    u_pd1 = JT_11*u_pdx + JT_12*u_pdy + JT_13*u_pdz;
    u_pd2 = JT_21*u_pdx + JT_22*u_pdy + JT_23*u_pdz;
    u_pd3 = JT_31*u_pdx + JT_32*u_pdy + JT_33*u_pdz;

    // Applied Force in Z-Direction
    u_app1 = 0.0254*JT_13*Fzcmd/Kt;
    u_app2 = 0.0254*JT_23*Fzcmd/Kt;
    u_app3 = 0.0254*JT_33*Fzcmd/Kt;



    // Friction Compensation
    if (Omega1 > minVel1) {
        u_fric1 = visc_pos1*Omega1 + coul_pos1;
    } else if (Omega1 < -minVel1) {
        u_fric1 = visc_neg1*Omega1 + coul_neg1;
    } else {
        u_fric1 = minSlope1*Omega1;
    }

    if (Omega2 > minVel2) {
        u_fric2 = visc_pos2*Omega2 + coul_pos2;
    } else if (Omega2 < -minVel2) {
        u_fric2 = visc_neg2*Omega2 + coul_neg2;
    } else {
        u_fric2 = minSlope2*Omega2;
    }

    if (Omega3 > minVel3) {
        u_fric3 = visc_pos3*Omega3 + coul_pos3;
    } else if (Omega3 < -minVel3) {
        u_fric3 = visc_neg3*Omega3 + coul_neg3;
    } else {
        u_fric3 = minSlope3*Omega3;
    }

    // Total Controller Output and Saturation

    tau_cmd1 = u_pd1 + fric_factor*u_fric1 + u_app1;
    if (tau_cmd1 > 5) {
        tau_cmd1 = 5;
    } else if (tau_cmd1 < -5) {
        tau_cmd1 = -5;
    }
    *tau1 = tau_cmd1;

    tau_cmd2 = u_pd2 + fric_factor*u_fric2 + u_app2;
    if (tau_cmd2 > 5) {
        tau_cmd2 = 5;
    } else if (tau_cmd2 < -5) {
        tau_cmd2 = -5;
    }
    *tau2 = tau_cmd2;

    tau_cmd3 = u_pd3 + fric_factor*u_fric3 + u_app3;
    if (tau_cmd3 > 5) {
        tau_cmd3 = 5;
    } else if (tau_cmd3 < -5) {
        tau_cmd3 = -5;
    }
    *tau3 = tau_cmd3;

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        if (whattoprint > 0.5) {
            serial_printf(&SerialA, "I love robotics\n\r");
        } else {
            printtheta1motor = px;
            printtheta2motor = py;
            printtheta3motor = pz;


            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }


    Simulink_PlotVar1 = xd;
    Simulink_PlotVar2 = yd;
    Simulink_PlotVar3 = zd;
    Simulink_PlotVar4 = 0;

    mycount++;
}



void printing(void){
    serial_printf(&SerialA, "x: %.2f, y: %.2f, z: %.2f  \n\r",printtheta1motor, printtheta2motor, printtheta3motor);
}
