#pragma once
#include "commdef.h"


class Crowncube
{
public:
	Crowncube() {};
	//���캯����ʼ���б�
	Crowncube(int _m_ID) :m_ID(_m_ID) {};
	~Crowncube() {};

public:
	//�ֿ���Ϣ
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
	//�ֿ�����Ϣ
	void set_cube_face();   //�����ڱ�ʶδȥ���շ���ǰ�����������������
	void set_face_id();     //��xyz_id �Ƕ�Ӧblock xyz_id�ֱ���x��y��zά��+-1���ɵ�
	void set_face_center(); //��������������
	void set_face_sign(int _face_id_x, int _face_id_y, int _face_id_z);  //������id���ø���Ϊ������
	                   

	bool* face_sign();
	Eigen::Vector3i* face_id();
	Eigen::Vector3f* face_center();
	Eigen::MatrixXf face_point(int _face_id);

public:
	//�ֿ鶥����Ϣ
	void set_point_xyz();

	Eigen::Vector3f* point_xyz();


private:
	//�ֿ���Ϣ
	int m_ID;      //�ֿ���
	int m_Pnum;    //�ֿ��ڵ�����(.PCD)
	double m_rPnum;  //�ֿ��ڵ�����ռ���ֵ����
	Eigen::Vector3i m_scale;   //xyz��ֿ����
	double m_size; //�ֿ�ߴ�
    Eigen::Vector3f m_center;  //�ֿ���������
    Eigen::Vector3f m_xyz_min; //�ֿ�������Сֵ
    Eigen::Vector3f m_xyz_max; //�ֿ��������ֵ
    Eigen::Vector3i m_xyz_id;  //����������루��1��ʼ��
	bool m_is_empty;  //��Ե�շֿ�
	double m_FAVD;    //�ֿ�Ҷ������ܶ�
	int m_scancnt;    //�ֿ鱻ɨ�����
	bool m_is_outer;  //�ǿ�����
	double FAVD_Weight;
	double Weight_total;

private:
	//�ֿ�����Ϣ
	Eigen::Vector3i m_face_id[6];
	//1-6���α�ʶ x_min x_max y_min y_max z_min z_max 6�����Ƿ�Ϊ������ trueΪ�����棬falseΪ��������
	bool m_face_sign[6];
	//�ֿ�����������
	Eigen::Vector3f m_face_center[6];


private:
	//�ֿ鶥����Ϣ
	//1-8���α�ʶ����(x,y,z)����(-,-,-)��(-,-,+)��(-,+,-)��(-,+,+)��(+,-,-)��(+,-,+)��(+,+,-)��(+,+,+)8����
	Eigen::Vector3f m_point_xyz[8];

};


