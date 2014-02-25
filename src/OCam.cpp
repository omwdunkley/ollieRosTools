/*------------------------------------------------------------------------------
   Code modified version of
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ollie_vo/OCam.hpp"

int get_ocam_model(struct ocam_model& myocam_model, const std::string &filename){
    //double *pol        = &(myocam_model.pol);
    //double *invpol     = &(myocam_model.invpol);
    double *xc         = &(myocam_model.xc);
    double *yc         = &(myocam_model.yc);
    double *c          = &(myocam_model.c);
    double *d          = &(myocam_model.d);
    double *e          = &(myocam_model.e);
    int    *width      = &(myocam_model.width);
    int    *height     = &(myocam_model.height);
    int *length_pol    = &(myocam_model.length_pol);
    int *length_invpol = &(myocam_model.length_invpol);
    FILE *f;
    char buf[CMV_MAX_BUF];
    int i;

    //Open file
    if(!(f=fopen(filename.c_str(),"r"))){
        printf("File %s cannot be opened\n", filename.c_str());
        return -1;
    }

    //Read polynomial coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", length_pol);
    for (i = 0; i < *length_pol; i++) {
        fscanf(f," %lf", &myocam_model.pol[i]);
    }

    //Read inverse polynomial coefficients
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d", length_invpol);
    for (i = 0; i < *length_invpol; i++) {
        fscanf(f," %lf",&myocam_model.invpol[i]);
    }

    //Read center coordinates
    fscanf(f,"\n");
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf\n", xc, yc);

    //Read affine coefficients
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%lf %lf %lf\n", c,d,e);

    //Read image size
    fgets(buf,CMV_MAX_BUF,f);
    fscanf(f,"\n");
    fscanf(f,"%d %d", height, width);

    fclose(f);
    return 0;
}

//------------------------------------------------------------------------------
cv::Point3d cam2world(const cv::Point2d& pixel, const struct ocam_model& m){
    double invdet  = 1/(m.c-m.d*m.e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

    double xp = invdet*(    (pixel.x - m.xc) - m.d*(pixel.y - m.yc) );
    double yp = invdet*( -m.e*(pixel.x - m.xc) + m.c*(pixel.y - m.yc) );

    double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
    double zp  = m.pol[0];
    double r_i = 1;
    int i;

    for (i = 1; i < m.length_pol; i++){
        r_i *= r;
        zp  += r_i*m.pol[i];
    }
    cv::Point3d point3d(xp, yp, zp);
    return point3d * (1./cv::norm(point3d));
}


cv::Point2d world2cam(const cv::Point3d& point3D, const struct ocam_model& m){

    double norm        = sqrt(point3D.x*point3D.x + point3D.y*point3D.y);
    double theta       = atan(point3D.z/norm);
    double t, t_i;
    double rho, x, y;
    double invnorm;
    int i;
    cv::Point2d pixel;

    if (norm != 0){
        invnorm = 1/norm;
        t  = theta;
        rho = m.invpol[0];
        t_i = 1;

        for (i = 1; i < m.length_invpol; i++){
            t_i *= t;
            rho += t_i*m.invpol[i];
        }

        x = point3D.x*invnorm*rho;
        y = point3D.y*invnorm*rho;

        pixel.x = x*m.c + y*m.d + m.xc;
        pixel.y = x*m.e + y   + m.yc;

    } else {
        pixel.x = m.xc;
        pixel.y = m.yc;
    }

    return pixel;
}


void create_perspecive_undistortion_LUT(cv::Mat& mx, cv::Mat& my, const struct ocam_model& m, const float sf){

    int width = mx.cols;
    int height = mx.rows;
    float Nxc = height/2.0;
    float Nyc = width/2.0;
    float Nz  = -width/sf;

    for (int i=0; i<height; i++){
        for (int j=0; j<width; j++){
            cv::Point2f p = world2cam(cv::Point3d(i-Nxc, j-Nyc, Nz), m);
            mx.at<float>(i,j) = p.x;
            my.at<float>(i,j) = p.y;
        }
    }
}
