#include "world2cam.h"

#include <string.h>

#include "rpoly.h"

#include "ocam_functions.h"

void omni3d2pixel( double &x, double &y, const Eigen::Vector3d &xx, int length_poly, double *ss, int width, int height )
{
	// convert 3D coordinates vector into 2D pixel coordinates
	double denom = sqrt( xx[0]*xx[0] + xx[1]*xx[1] );
	double m = xx[2] / denom;

	double rho = 0;

	double *op = (double*) malloc( sizeof(double)*length_poly );
	for ( int i = 0; i < length_poly; i++ ) op[i] = ss[ length_poly - 1 - i ];
	op[length_poly - 2] = op[length_poly - 2] - m;

	int degree = length_poly - 1;

	double zeror[MAXDEGREE];
	double zeroi[MAXDEGREE];
	
	rpoly_ak1( op, &degree, zeror, zeroi );
    free(op);

	for ( int i = 0; i < degree; i++ ) {
		if ( zeroi[i] == 0 && zeror[i] > 0 && zeror[i] < height ) {
			rho = zeror[i];
			break;
		}
	}

	x = xx[0] / denom * rho ;
	y = xx[1] / denom * rho ;
}

void myworld2cam(cv::Vec2d &point2D, const Eigen::Vector3d &point3D, struct ocam_model *myocam_model)
{
	double *ss         = myocam_model->pol;
	int length_poly    = myocam_model->length_pol;
	double xc          = (myocam_model->xc);
	double yc          = (myocam_model->yc);
	double c           = (myocam_model->c);
	double d           = (myocam_model->d);
	double e           = (myocam_model->e);
	int    width       = (myocam_model->width);
	int    height      = (myocam_model->height);
	double x, y;
	omni3d2pixel( x, y, point3D, length_poly, ss, width, height );
	point2D[1] = x*c + y*d + xc;
	point2D[0] = x*e + y   + yc;
}

// this doesn't seem to be accurate
void world2cam_fast(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
	double *invpol     = myocam_model->invpol;
	double xc          = (myocam_model->xc);
	double yc          = (myocam_model->yc);
	double c           = (myocam_model->c);
	double d           = (myocam_model->d);
	double e           = (myocam_model->e);
	int    width       = (myocam_model->width);
	int    height      = (myocam_model->height);
	int length_invpol  = (myocam_model->length_invpol);
	double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
	double theta       = atan(point3D[2]/norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	if (norm != 0)  {
		invnorm = 1/norm;
		t  = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i*invpol[i];
		}

		x = point3D[0]*invnorm*rho;
		y = point3D[1]*invnorm*rho;

		point2D[0] = x*c + y*d + xc;
		point2D[1] = x*e + y   + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}
