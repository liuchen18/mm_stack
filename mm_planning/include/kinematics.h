#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <chrono>   

#define PI 3.1415926535897932

double PLimit[7]={PI*170/180,PI*120/180,PI*170/180,PI*120/180,PI*170/180,PI*120/180,PI*170/180};

using namespace std;
using namespace cv;
using namespace chrono;


int m3m3(double (&m3m3result)[3][3],double m3a[][3],double m3b[][3]);
int m31m13(double (&m31m13result)[3][3],double m31[],double m13[]);
double norm(double matrix_3[]);
int Inv_Kine(double (&Theta)[7],Mat RPY_p,double gamma,double PreTheta[],int PreSize);
int sign(double x);
//int testpart();
int Test();
int m_transpose(double (&m_trans_result)[3][3],double i_matrix[][3]);
int For_kine(double (&f_kineresult)[4][4],double InTheta[7]);

Mat RPY2r(Mat RPY);

int Inv_Kine(double (&Theta)[7],Mat p_RPY,double gamma,double PreTheta[],int PreSize)
{
	double T_taj[4][4]={0};
	Mat RPY = Mat::zeros(3,1,CV_32FC1);
	Mat p = Mat::zeros(3,1,CV_32FC1);
	for (int i=0;i<3;i++)
	{
		p.ptr<float>(i)[0]=p_RPY.ptr<float>(i)[0];
		RPY.ptr<float>(i)[0]=p_RPY.ptr<float>(i+3)[0];
	}
	Mat R = RPY2r(RPY);
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			T_taj[i][j] = R.ptr<float>(i)[j];
		}
	}
	for (int i=0;i<3;i++)
	{
		T_taj[i][3] = p.ptr<float>(i)[0];
	}
	T_taj[3][3]=0;

	/*for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			cout<<setw(20)<<T_taj[i][j];
		}
		cout<<endl;
	}*/

	double nx=T_taj[0][0]; double ny=T_taj[1][0]; double nz=T_taj[2][0];
    double ox=T_taj[0][1]; double oy=T_taj[1][1]; double oz=T_taj[2][1];
    double ax=T_taj[0][2]; double ay=T_taj[1][2]; double az=T_taj[2][2];
	double px=T_taj[0][3]; double py=T_taj[1][3]; double pz=T_taj[2][3];
	double R07_d[3][3]={{nx,ox,ax},{ny,oy,ay},{nz,oz,az}};

	double d[7]={360,0,420,0,400,0,206};
	double xsw[3]={0};xsw[0]=px-d[6]*ax;xsw[1]=py-d[6]*ay;xsw[2]=pz-d[0]-d[6]*az;	
	double usw[3]={0};usw[0]=xsw[0]/norm(xsw);usw[1]=xsw[1]/norm(xsw);usw[2]=xsw[2]/norm(xsw);
	double R_usw[3][3]={{0,-usw[2],usw[1]},{usw[2],0,-usw[0]},{-usw[1],usw[0],0}};

	double theta4[8]={0};
	theta4[0]=-acos((pow(norm(xsw),2)-pow(d[2],2)-pow(d[4],2))/2/d[2]/d[4]);
	theta4[1]=theta4[0];theta4[2]=theta4[0];theta4[3]=theta4[0];
	theta4[4]=-theta4[0];theta4[5]=-theta4[0];theta4[6]=-theta4[0];theta4[7]=-theta4[0];

	double AA=d[2]/d[4]+cos(-theta4[0]);double BB=sin(-theta4[0]);                         //theta2��һ�����
	double theta2_0=acos(xsw[2]/d[4]/sqrt(pow(AA,2)+pow(BB,2)))-atan2(BB,AA);
	double CC=sign(d[4]*sin(theta2_0 - theta4[0])+d[2]*sin(theta2_0));
	double theta1_0=atan2(CC*xsw[1],CC*xsw[0]);

	double R03_o[3][3]={cos(theta1_0)*cos(theta2_0), -cos(theta1_0)*sin(theta2_0), -sin(theta1_0),
        cos(theta2_0)*sin(theta1_0), -sin(theta1_0)*sin(theta2_0),  cos(theta1_0),
		-sin(theta2_0),             -cos(theta2_0),            0};
	double As[3][3]={0};double Bs[3][3]={0};double Cs[3][3]={0};
	m3m3(As,R_usw,R03_o);
	double temp_Bs[3][3]={0};double temp_Cs[3][3]={0};double minusR_usw[3][3]={0};

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			minusR_usw[i][j]=-R_usw[i][j];

	m3m3(temp_Bs,minusR_usw,R_usw);m3m3(Bs,temp_Bs,R03_o);
	m31m13(temp_Cs,usw,usw);m3m3(Cs,temp_Cs,R03_o);         

	double theta2[8]={0};                            //theta2���,theta4�����������Ӧtheta2���������
	theta2[0]=acos(-As[2][1]*sin(gamma)-Bs[2][1]*cos(gamma)-Cs[2][1]);
	theta2[1]=theta2[0];theta2[2]=-theta2[0];theta2[3]=-theta2[0];                //theta2[0]-theta2[3]

	AA=d[2]/d[4]+cos(theta4[4]);BB=sin(theta4[4]);                                  //theta2�ڶ������
	theta2_0=acos(xsw[2]/d[4]/sqrt(pow(AA,2)+pow(BB,2)))-atan2(BB,AA);
	CC=sign(d[4]*sin(theta2_0 + theta4[4])+d[2]*sin(theta2_0));
	theta1_0=atan2(CC*xsw[1],CC*xsw[0]);

	double R03_o2[3][3]={cos(theta1_0)*cos(theta2_0), -cos(theta1_0)*sin(theta2_0), -sin(theta1_0),
        cos(theta2_0)*sin(theta1_0), -sin(theta1_0)*sin(theta2_0),  cos(theta1_0),
		-sin(theta2_0),             -cos(theta2_0),            0};
	double As_2[3][3]={0};double Bs_2[3][3]={0};double Cs_2[3][3]={0};
	m3m3(As_2,R_usw,R03_o2);
	double temp_Bs2[3][3]={0};double temp_Cs2[3][3]={0};

	m3m3(temp_Bs2,minusR_usw,R_usw);m3m3(Bs_2,temp_Bs2,R03_o2);
	m31m13(temp_Cs2,usw,usw);m3m3(Cs_2,temp_Cs2,R03_o2);         //temp������Ҫע�⣻

	theta2[4]=acos(-As_2[2][1]*sin(gamma)-Bs_2[2][1]*cos(gamma)-Cs_2[2][1]);
	theta2[5]=theta2[4];theta2[6]=-theta2[4];theta2[7]=-theta2[4];                    //theta2[4]-theta2[7]


	double theta1[8]={0};                            //theta1���
	theta1[0]=atan2(sign(sin(theta2[0]))*(-As[1][1]*sin(gamma)-Bs[1][1]*cos(gamma)-Cs[1][1]),
		sign(sin(theta2[0]))*(-As[0][1]*sin(gamma)-Bs[0][1]*cos(gamma)-Cs[0][1]));
	theta1[1]=theta1[0];
	theta1[2]=atan2(sign(sin(theta2[2]))*(-As[1][1]*sin(gamma)-Bs[1][1]*cos(gamma)-Cs[1][1]),
		sign(sin(theta2[2]))*(-As[0][1]*sin(gamma)-Bs[0][1]*cos(gamma)-Cs[0][1]));
    theta1[3]=theta1[2];
    theta1[4]=atan2(sign(sin(theta2[4]))*(-As_2[1][1]*sin(gamma)-Bs_2[1][1]*cos(gamma)-Cs_2[1][1]),
		sign(sin(theta2[4]))*(-As_2[0][1]*sin(gamma)-Bs_2[0][1]*cos(gamma)-Cs_2[0][1]));
	theta1[5]=theta1[4];
    theta1[6]=atan2(sign(sin(theta2[6]))*(-As_2[1][1]*sin(gamma)-Bs_2[1][1]*cos(gamma)-Cs_2[1][1]),
		sign(sin(theta2[6]))*(-As_2[0][1]*sin(gamma)-Bs_2[0][1]*cos(gamma)-Cs_2[0][1]));
	theta1[7]=theta1[6];

	double theta3[8]={0};                           //theta3���
	theta3[0]=atan2(sign(sin(theta2[0]))*(As[2][2]*sin(gamma)+Bs[2][2]*cos(gamma)+Cs[2][2]),
		sign(sin(theta2[0]))*(-As[2][0]*sin(gamma)-Bs[2][0]*cos(gamma)-Cs[2][0]));
	theta3[1]=theta3[0];
	theta3[2]=atan2(sign(sin(theta2[2]))*(As[2][2]*sin(gamma)+Bs[2][2]*cos(gamma)+Cs[2][2]),
		sign(sin(theta2[2]))*(-As[2][0]*sin(gamma)-Bs[2][0]*cos(gamma)-Cs[2][0]));
	theta3[3]=theta3[2];
	theta3[4]=atan2(sign(sin(theta2[4]))*(As_2[2][2]*sin(gamma)+Bs_2[2][2]*cos(gamma)+Cs_2[2][2]),
		sign(sin(theta2[4]))*(-As_2[2][0]*sin(gamma)-Bs_2[2][0]*cos(gamma)-Cs_2[2][0]));
	theta3[5]=theta3[4];
	theta3[6]=atan2(sign(sin(theta2[6]))*(As_2[2][2]*sin(gamma)+Bs_2[2][2]*cos(gamma)+Cs_2[2][2]),
		sign(sin(theta2[6]))*(-As_2[2][0]*sin(gamma)-Bs_2[2][0]*cos(gamma)-Cs_2[2][0]));
	theta3[7]=theta3[6];

    double R34_1[3][3] ={cos(-theta4[0]), 0,  sin(-theta4[0]),
        sin(-theta4[0]), 0, -cos(-theta4[0]),
		0, 1,            0 };
	double R34_2[3][3]={cos(-theta4[4]), 0,  sin(-theta4[4]),
        sin(-theta4[4]), 0, -cos(-theta4[4]),
		0, 1,            0 };

	double trans_R34_1[3][3]={0};double trans_R34_2[3][3]={0};
	double trans_As[3][3]={0};double trans_Bs[3][3]={0};double trans_Cs[3][3]={0};
	double trans_As2[3][3]={0};double trans_Bs2[3][3]={0};double trans_Cs2[3][3]={0};
	m_transpose(trans_R34_1,R34_1);m_transpose(trans_R34_2,R34_2);
	m_transpose(trans_As,As);m_transpose(trans_Bs,Bs);m_transpose(trans_Cs,Cs);
	m_transpose(trans_As2,As_2);m_transpose(trans_Bs2,Bs_2);m_transpose(trans_Cs2,Cs_2);



	double Aw_1[3][3]={0};double Bw_1[3][3]={0};double Cw_1[3][3]={0};
	double temp_Aw1[3][3]={0};double temp_Bw1[3][3]={0};double temp_Cw1[3][3]={0};
    m3m3(temp_Aw1,trans_R34_1,trans_As);m3m3(Aw_1,temp_Aw1,R07_d);
	m3m3(temp_Bw1,trans_R34_1,trans_Bs);m3m3(Bw_1,temp_Bw1,R07_d);
	m3m3(temp_Cw1,trans_R34_1,trans_Cs);m3m3(Cw_1,temp_Cw1,R07_d);

	double Aw_2[3][3]={0};double Bw_2[3][3]={0};double Cw_2[3][3]={0};
	double temp_Aw2[3][3]={0};double temp_Bw2[3][3]={0};double temp_Cw2[3][3]={0};
    m3m3(temp_Aw2,trans_R34_2,trans_As2);m3m3(Aw_2,temp_Aw2,R07_d);
	m3m3(temp_Bw2,trans_R34_2,trans_Bs2);m3m3(Bw_2,temp_Bw2,R07_d);
	m3m3(temp_Cw2,trans_R34_2,trans_Cs2);m3m3(Cw_2,temp_Cw2,R07_d);
	
	double theta6[8]={0};                               //theta6���
	theta6[0]=acos(Aw_1[2][2]*sin(gamma)+Bw_1[2][2]*cos(gamma)+Cw_1[2][2]);
	theta6[1]=-theta6[0];theta6[2]=theta6[0];theta6[3]=theta6[1];

	theta6[4]=acos(Aw_2[2][2]*sin(gamma)+Bw_2[2][2]*cos(gamma)+Cw_2[2][2]);
	theta6[5]=-theta6[4];theta6[6]=theta6[4];theta6[7]=theta6[5];

	double theta5[8]={0};                               //theta5���
	theta5[0]=atan2(sign(sin(theta6[0]))*(Aw_1[1][2]*sin(gamma)+Bw_1[1][2]*cos(gamma)+Cw_1[1][2]),
		sign(sin(theta6[0]))*(Aw_1[0][2]*sin(gamma)+Bw_1[0][2]*cos(gamma)+Cw_1[0][2]));
	theta5[1]=atan2(sign(sin(theta6[1]))*(Aw_1[1][2]*sin(gamma)+Bw_1[1][2]*cos(gamma)+Cw_1[1][2]),
		sign(sin(theta6[1]))*(Aw_1[0][2]*sin(gamma)+Bw_1[0][2]*cos(gamma)+Cw_1[0][2]));
	theta5[2]=theta5[0];theta5[3]=theta5[1];
	
	theta5[4]=atan2(sign(sin(theta6[6]))*(Aw_2[1][2]*sin(gamma)+Bw_2[1][2]*cos(gamma)+Cw_2[1][2]),
		sign(sin(theta6[6]))*(Aw_2[0][2]*sin(gamma)+Bw_2[0][2]*cos(gamma)+Cw_2[0][2]));
	theta5[5]=atan2(sign(sin(theta6[7]))*(Aw_2[1][2]*sin(gamma)+Bw_2[1][2]*cos(gamma)+Cw_2[1][2]),
		sign(sin(theta6[7]))*(Aw_2[0][2]*sin(gamma)+Bw_2[0][2]*cos(gamma)+Cw_2[0][2]));
	theta5[6]=theta5[4];theta5[7]=theta5[5];

	double theta7[8]={0};
	theta7[0]=atan2(sign(sin(theta6[0]))*(Aw_1[2][1]*sin(gamma)+Bw_1[2][1]*cos(gamma)+Cw_1[2][1]),
		sign(sin(theta6[0]))*(-Aw_1[2][0]*sin(gamma)-Bw_1[2][0]*cos(gamma)-Cw_1[2][0]));
	theta7[1]=atan2(sign(sin(theta6[1]))*(Aw_1[2][1]*sin(gamma)+Bw_1[2][1]*cos(gamma)+Cw_1[2][1]),
		sign(sin(theta6[1]))*(-Aw_1[2][0]*sin(gamma)-Bw_1[2][0]*cos(gamma)-Cw_1[2][0]));
	theta7[2]=theta7[0];theta7[3]=theta7[1];
	
	theta7[4]=atan2(sign(sin(theta6[6]))*(Aw_2[2][1]*sin(gamma)+Bw_2[2][1]*cos(gamma)+Cw_2[2][1]),
		sign(sin(theta6[6]))*(-Aw_2[2][0]*sin(gamma)-Bw_2[2][0]*cos(gamma)-Cw_2[2][0]));
	theta7[5]=atan2(sign(sin(theta6[7]))*(Aw_2[2][1]*sin(gamma)+Bw_2[2][1]*cos(gamma)+Cw_2[2][1]),
		sign(sin(theta6[7]))*(-Aw_2[2][0]*sin(gamma)-Bw_2[2][0]*cos(gamma)-Cw_2[2][0]));
	theta7[6]=theta7[4];theta7[7]=theta7[5];

	double eightresult[8][7]={0};
	for(int i=0;i<8;i++)
		eightresult[i][0]=theta1[i];
	for(int i=0;i<8;i++)
		eightresult[i][1]=theta2[i];
	for(int i=0;i<8;i++)
		eightresult[i][2]=theta3[i];
	for(int i=0;i<8;i++)
		eightresult[i][3]=theta4[i];
	for(int i=0;i<8;i++)
		eightresult[i][4]=theta5[i];
	for(int i=0;i<8;i++)
		eightresult[i][5]=theta6[i];
	for(int i=0;i<8;i++)
		eightresult[i][6]=theta7[i];

	int flag1=0;
		double Min=100;
		for (int i=0;i<8;i++)
		{
			double SelectSolution=0;
			for (int j=0;j<7;j++)
			{
				SelectSolution=SelectSolution+pow((eightresult[i][j]-PreTheta[j]),2);
			}
			if (SelectSolution<Min)
			{
				Min=SelectSolution;
				flag1=i;
			}
			//cout<<i<<" "<<SelectSolution<<endl;
			//cout<<eightresult[i][0]*180/PI<<"/"<<eightresult[i][1]*180/PI<<"/"<<eightresult[i][2]*180/PI<<"/"<<eightresult[i][3]*180/PI<<"/"<<eightresult[i][4]*180/PI<<"/"<<eightresult[i][5]*180/PI<<"/"<<eightresult[i][6]*180/PI<<"/"<<endl;
		}
		for (int i=0;i<7;i++)
		{
			Theta[i]=eightresult[flag1][i];
		}

	for (int i=0;i<7;i++)
	{
		//cout<<"i: "<<i<<" "<<Theta[i]<<endl;
		if ((Theta[i]>=PLimit[i])||(Theta[i]<=-PLimit[i]))
		{
			
			cout<<" PLimit Wrong "<<endl;
			exit(0);
		}
	}
	return 0;
}

int For_kine(double (&Final)[4][4],double InTheta[7])
{
	InTheta[3] = -InTheta[3];//iiwa speical DH parameters in axis 4
    double A1[4][4]={{cos(InTheta[0]),0,-sin(InTheta[0]),0},{sin(InTheta[0]),0,cos(InTheta[0]),0},{0,-1,0,360},{0,0,0,1}};
	double A2[4][4]={{cos(InTheta[1]),0,sin(InTheta[1]),0},{sin(InTheta[1]),0,-cos(InTheta[1]),0},{0,1,0,0},{0,0,0,1}};
	double A3[4][4]={{cos(InTheta[2]),0,-sin(InTheta[2]),0},{sin(InTheta[2]),0,cos(InTheta[2]),0},{0,-1,0,420},{0,0,0,1}};
	double A4[4][4]={{cos(InTheta[3]),0,sin(InTheta[3]),0},{sin(InTheta[3]),0,-cos(InTheta[3]),0},{0,1,0,0},{0,0,0,1}};
	double A5[4][4]={{cos(InTheta[4]),0,sin(-InTheta[4]),0},{sin(InTheta[4]),0,cos(InTheta[4]),0},{0,-1,0,400},{0,0,0,1}};
	double A6[4][4]={{cos(InTheta[5]),0,sin(InTheta[5]),0},{sin(InTheta[5]),0,-cos(InTheta[5]),0},{0,1,0,0},{0,0,0,1}};
	double A7[4][4]={{cos(InTheta[6]),-sin(InTheta[6]),0,0},{sin(InTheta[6]),cos(InTheta[6]),0,0},{0,0,1,206},{0,0,0,1}};
	double A_1[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				A_1[i][j]+=A1[i][p]*A2[p][j];
			}
		}
	}
    double A_2[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				A_2[i][j]+=A_1[i][p]*A3[p][j];
			}
		}
	}
    double A_3[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				A_3[i][j]+=A_2[i][p]*A4[p][j];
			}
		}
	}
    double A_4[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				A_4[i][j]+=A_3[i][p]*A5[p][j];
			}
		}
	}
    double A_5[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				A_5[i][j]+=A_4[i][p]*A6[p][j];
			}
		}
	}
    double T[4][4]={0};
    for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			for(int p=0;p<4;p++)
			{
				T[i][j]+=A_5[i][p]*A7[p][j];
			}
		}
	}
	for (int m=0;m<4;m++){
		for (int n=0;n<4;n++)
			Final[m][n]=T[m][n];
	}
	//for(int n=0;n<3;n++){                     //��һ��
	//	double tempx=0;
	//	for(int m=0;m<3;m++){
	//		tempx=tempx+pow(Final[m][n],2);
	//	}
	//	for(int m=0;m<3;m++){
	//		Final[m][n]=Final[m][n]/tempx;
	//	}
	//}
	return 0;
}

int m3m3(double (&m3m3result)[3][3],double m3a[][3],double m3b[][3])
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			m3m3result[i][j]=0;
			for(int m=0;m<3;m++)
			{
				m3m3result[i][j]=m3m3result[i][j]+m3a[i][m]*m3b[m][j];
			}
		}
	}
	return 0;
}

int m31m13(double (&m31m13result)[3][3],double m31[],double m13[])
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
            m31m13result[i][j]=m31[i]*m13[j];
		}
	}
	return 0;
}

double norm(double matrix_3[])
{
	double norm=0;
	norm=sqrt(pow(matrix_3[0],2)+pow(matrix_3[1],2)+pow(matrix_3[2],2));
	return norm;
}

int sign(double x)
{
    int y = 0;
    if (x >0)
    {
        y = 1;
    }
    else if (0 == x)
    {
        y = 0;
    }
    else if (x < 0)
    {
      y = -1;
    }
    return y;
}


int m_transpose(double (&m_trans_result)[3][3],double i_matrix[][3])
{
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			m_trans_result[i][j]=i_matrix[j][i];
	return 0;
}


Mat RPY2r(Mat RPY)
{
	float beta=RPY.ptr<float>(1)[0];    //beta
	float gamma=RPY.ptr<float>(0)[0];   //gamma
	float alpha=RPY.ptr<float>(2)[0];   //alpha
	Mat R=Mat::zeros(3,3,CV_32FC1);
	R.ptr<float>(0)[0]=cos(alpha)*cos(beta);
	R.ptr<float>(1)[0]=sin(alpha)*cos(beta);
	R.ptr<float>(2)[0]=-sin(beta);
	R.ptr<float>(0)[1]=cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma);
	R.ptr<float>(1)[1]=sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma);
	R.ptr<float>(2)[1]=cos(beta)*sin(gamma);
	R.ptr<float>(0)[2]=cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);
	R.ptr<float>(1)[2]=sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
	R.ptr<float>(2)[2]=cos(beta)*cos(gamma);

	return R;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Mat r2RPY(Mat &R)
{
    
	float sy = sqrt(R.ptr<float>(0)[0] * R.ptr<float>(0)[0] +  R.ptr<float>(1)[0] * R.ptr<float>(1)[0]);
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
		//cout<<"!singular"<<endl;
        x = atan2(R.ptr<float>(2)[1] , R.ptr<float>(2)[2]);
        y = atan2(-R.ptr<float>(2)[0], sy);
        z = atan2(R.ptr<float>(1)[0], R.ptr<float>(0)[0]);
    }
    else
    {
		//cout<<"singular"<<endl;
        x = atan2(-R.ptr<float>(1)[2], R.ptr<float>(1)[1]);
        y = atan2(-R.ptr<float>(2)[0], sy);
        z = 0;
    }

	Mat RPY=Mat::zeros(3,1,CV_32FC1);
	RPY.ptr<float>(0)[0]=x;
	RPY.ptr<float>(0)[1]=y;
	RPY.ptr<float>(0)[2]=z;
    return RPY;
     
}