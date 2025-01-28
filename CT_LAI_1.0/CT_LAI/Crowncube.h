#pragma once
#include "commdef.h"


class Crowncube
{
public:
	Crowncube() {};
	//构造函数初始化列表
	Crowncube(int _m_ID) :m_ID(_m_ID) {};
	~Crowncube() {};

public:
	//分块信息
	void set_ID(int _m_ID);
	//int  xyz2ID(Eigen::Vector3i);
	void set_scale(int _scalex, int _scaley, int _scalez);
	void set_size(double _m_size);
	void set_center();
	void set_center(Eigen::Vector3f _m_center);
	void set_center(Eigen::Vector3f _m_xyz_min, Eigen::Vector3f _m_xyz_max);
	void set_xyz_range(Eigen::Vector3f _m_xyz_min, Eigen::Vector3f _m_xyz_max);
	void set_xyz_range(double _m_x_min, double _m_y_min, double _m_z_min, double _block_size);
	void set_xyz_id(int _m_xyz_id_x, int _m_xyz_id_y, int _m_xyz_id_z);
	void set_FAVD(double _FAVD);
	void set_scancnt(int _scancnt);
	void set_is_empty(bool _is_empty);
	void set_is_outer(bool _is_outer);
	void set_FAVD_W(double _FAVD_Weight);
	void set_Weight_T(double _Weight_T);
	void set_Pnum(int _Pnum);
	void set_rPnum(double r_Pnum);
	void add_FAVD_W(double _FAVD_W);
	void add_Weight_T(double _Weihgt);

	Eigen::Vector3f center();
	Eigen::Vector3f xyz_min();
	Eigen::Vector3f xyz_max();
	Eigen::Vector3i xyz_id();
	int ID();
	Eigen::Vector3i scale();
	double size();
	double FAVD();
	int scancnt();
	bool is_empty();
	bool is_outer();
	int Pnum();
	double rPnum();
	
public:
	//分块面信息
	void set_cube_face();   //仅用于标识未去除空方块前所有立方体的外向面
	void set_face_id();     //面xyz_id 是对应block xyz_id分别沿x，y，z维上+-1生成的
	void set_face_center(); //设置面中心坐标
	void set_face_sign(int _face_id_x, int _face_id_y, int _face_id_z);  //根据面id设置该面为外向面
	                   

	bool* face_sign();
	Eigen::Vector3i* face_id();
	Eigen::Vector3f* face_center();
	Eigen::MatrixXf face_point(int _face_id);

public:
	//分块顶点信息
	void set_point_xyz();

	Eigen::Vector3f* point_xyz();


private:
	//分块信息
	int m_ID;      //分块编号
	int m_Pnum;    //分块内点云数(.PCD)
	double m_rPnum;  //分块内点云数占最大值比例
	Eigen::Vector3i m_scale;   //xyz轴分块个数
	double m_size; //分块尺寸
    Eigen::Vector3f m_center;  //分块中心坐标
    Eigen::Vector3f m_xyz_min; //分块三轴最小值
    Eigen::Vector3f m_xyz_max; //分块三轴最大值
    Eigen::Vector3i m_xyz_id;  //方块三向编码（从1开始）
	bool m_is_empty;  //边缘空分块
	double m_FAVD;    //分块叶面积体密度
	int m_scancnt;    //分块被扫描次数
	bool m_is_outer;  //非空外层块
	double FAVD_Weight;
	double Weight_total;

private:
	//分块面信息
	Eigen::Vector3i m_face_id[6];
	//1-6依次标识 x_min x_max y_min y_max z_min z_max 6个面是否为外向面 true为外向面，false为非外向面
	bool m_face_sign[6];
	//分块面中心坐标
	Eigen::Vector3f m_face_center[6];


private:
	//分块顶点信息
	//1-8依次标识沿着(x,y,z)三轴(-,-,-)、(-,-,+)、(-,+,-)、(-,+,+)、(+,-,-)、(+,-,+)、(+,+,-)、(+,+,+)8个点
	Eigen::Vector3f m_point_xyz[8];

};


