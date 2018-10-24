#include "math.h";


void CrossProduct(double a[3], double b[3], double ret[3])
{
    ret[0] = a[1] * b[2] - a[2] * b[1];
    ret[1] = a[2] * b[0] - a[0] * b[2];
    ret[2] = a[0] * b[1] - a[1] * b[0];
}

double DotProduct(double a[3], double b[3])
{
    double result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    return result;
}

double Normalize(double v[3])
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return result;
}

void Rotate(double** rotateMatrix, double u[3], double ret[3]){
    ret[0]=rotateMatrix[0][0]*u[0]+rotateMatrix[0][1]*u[1]+rotateMatrix[0][2]*u[2];
    ret[1]=rotateMatrix[1][0]*u[0]+rotateMatrix[1][1]*u[1]+rotateMatrix[1][2]*u[2];
    ret[2]=rotateMatrix[2][0]*u[0]+rotateMatrix[2][1]*u[1]+rotateMatrix[2][2]*u[2];
}

void RotationMatrix(double angle, double u[3], double rotatinMatrix[3][3])
{
    double norm = Normalize(u);
    
    u[0] = u[0] / norm;
    u[1] = u[1] / norm;
    u[2] = u[2] / norm;

    rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
    rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle) - u[2] * sin(angle));
    rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

    rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
      
    rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

}

//cal transport
void Calculation3d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[3][3])
{
    double  rotationAxis[3];
    double rotationAngle;
    CrossProduct(vectorBefore, vectorAfter, rotationAxis);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    RotationMatrix(rotationAngle, rotationAxis, rotatinMatrix);
}

void Calculation4d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[4][4])
{
    double rotate3d[3][3];
	Calculation3d(vectorBefore,vectorAfter, rotate3d);

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			rotatinMatrix[i][j] = rotate3d[i][j];

	for(int i = 0; i < 3; i++)
	{
		rotatinMatrix[i][3] = 0;
		rotatinMatrix[3][i] = 0;
	}

	rotatinMatrix[3][3] = 1;
}
