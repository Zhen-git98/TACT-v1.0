#include "Crowncube.h"

//方块赋值
void Crowncube::set_ID(int _m_ID)
{
	this->m_ID = _m_ID;
}

void Crowncube::set_scale(int _scalex, int _scaley, int _scalez)
{
	this->m_scale[0] = _scalex;
	this->m_scale[1] = _scaley;
	this->m_scale[2] = _scalez;
}

void Crowncube::set_size(double _m_size)
{
	this->m_size = _m_size;
}

void Crowncube::set_center()
{
	this->m_center[0] = (this->m_xyz_min[0] + this->m_xyz_max[0]) / 2.0;
	this->m_center[1] = (this->m_xyz_min[1] + this->m_xyz_max[1]) / 2.0;
	this->m_center[2] = (this->m_xyz_min[2] + this->m_xyz_max[2]) / 2.0;
}

void Crowncube::set_center(Eigen::Vector3f _m_center)
{
	this->m_center = _m_center;
}

void Crowncube::set_center(Eigen::Vector3f _m_xyz_min, Eigen::Vector3f _m_xyz_max)
{
	this->m_center[0] = (_m_xyz_min[0] + _m_xyz_max[0]) / 2.0;
	this->m_center[1] = (_m_xyz_min[1] + _m_xyz_max[1]) / 2.0;
	this->m_center[2] = (_m_xyz_min[2] + _m_xyz_max[2]) / 2.0;
}

void Crowncube::set_xyz_range(Eigen::Vector3f _m_xyz_min, Eigen::Vector3f _m_xyz_max)
{
	this->m_xyz_min = _m_xyz_min;
	this->m_xyz_max = _m_xyz_max;
}

void Crowncube::set_xyz_range(double _m_x_min, double _m_y_min, double _m_z_min, double _block_size)
{
	this->m_xyz_min[0] = _m_x_min;
	this->m_xyz_min[1] = _m_y_min;
	this->m_xyz_min[2] = _m_z_min;
	this->m_xyz_max[0] = _m_x_min + _block_size;
	this->m_xyz_max[1] = _m_y_min + _block_size;
	this->m_xyz_max[2] = _m_z_min + _block_size;
}

void Crowncube::set_xyz_id(int _m_xyz_id_x, int _m_xyz_id_y, int _m_xyz_id_z)
{
	this->m_xyz_id[0] = _m_xyz_id_x;
	this->m_xyz_id[1] = _m_xyz_id_y;
	this->m_xyz_id[2] = _m_xyz_id_z;
}

void Crowncube::set_is_empty(bool _is_empty)
{
	this->m_is_empty = _is_empty;
}

void Crowncube::set_FAVD(double _FAVD)
{
	this->m_FAVD = _FAVD;
}

void Crowncube::set_scancnt(int _scancnt)
{
	this->m_scancnt = _scancnt;
}

void Crowncube::set_is_outer(bool _is_outer)
{
	//XYZ20230821 outer只是总块外层意义不大，应该是去除空块后最外一层
	/*
	this->m_is_outer = false;
	int cnt = 0;
	this->m_xyz_id[0] == 1 || this->m_xyz_id[0] == this->m_scale[0] ? cnt++ : cnt;
	this->m_xyz_id[1] == 1 || this->m_xyz_id[1] == this->m_scale[1] ? cnt++ : cnt;
	this->m_xyz_id[2] == 1 || this->m_xyz_id[2] == this->m_scale[2] ? cnt++ : cnt;
	if (cnt >= 1)
	{
		this->m_is_outer = true;
	}
	*/
	this->m_is_outer = _is_outer;
}

void Crowncube::set_FAVD_W(double _FAVD_Weight)
{
	this->FAVD_Weight = _FAVD_Weight;
}

void Crowncube::set_Weight_T(double _Weight_total)
{
	this->Weight_total = _Weight_total;
}


void Crowncube::add_FAVD_W(double _FAVD_W)
{
	this->FAVD_Weight += _FAVD_W;
}
void Crowncube::add_Weight_T(double _Weihgt)
{
	this->Weight_total += _Weihgt;
}

void Crowncube::set_Pnum(int _Pnum)
{
	this->m_Pnum = _Pnum;
}

void Crowncube::set_rPnum(double _rPnum)
{
	this->m_rPnum = _rPnum;
}


//方块信息
Eigen::Vector3f Crowncube::center()
{
	return this->m_center;
}

Eigen::Vector3f Crowncube::xyz_min()
{
	return this->m_xyz_min;
}

Eigen::Vector3f Crowncube::xyz_max()
{
	return this->m_xyz_max;
}

Eigen::Vector3i Crowncube::xyz_id()
{
	return this->m_xyz_id;
}

int Crowncube::ID()
{
	return this->m_ID;
}

//int Crowncube::xyz2ID(Eigen::Vector3i _xyz_id)
//{
	//int scale = this->m_scale;
	//int ID = (_xyz_id[0] - 1) + (_xyz_id[1] - 1) * scale + (_xyz_id[2] - 1) * pow(scale, 2);
	//return ID;
//}

Eigen::Vector3i Crowncube::scale()
{
	return this->m_scale;
}

double Crowncube::size()
{
	return this->m_size;
}

double Crowncube::FAVD()
{
	if (this->Weight_total > 0.0)
		this->m_FAVD = this->FAVD_Weight / this->Weight_total;
	else
		this->m_FAVD = 0.0;
	return this->m_FAVD;
}

int Crowncube::scancnt()
{
	return this->m_scancnt;
}

bool Crowncube::is_empty()
{
	return this->m_is_empty;
}

bool Crowncube::is_outer()
{
	return this->m_is_outer;
}

int Crowncube::Pnum()
{
	return this->m_Pnum;
}

double Crowncube::rPnum()
{
	return this->m_rPnum;
}


//方块面赋值
void Crowncube::set_cube_face()
{
	this->m_xyz_id[0] == 1 ? this->m_face_sign[0] = true : this->m_face_sign[0] = false;
	this->m_xyz_id[0] == this->m_scale[0] ? this->m_face_sign[1] = true : this->m_face_sign[1] = false;
	this->m_xyz_id[1] == 1 ? this->m_face_sign[2] = true : this->m_face_sign[2] = false;
	this->m_xyz_id[1] == this->m_scale[1] ? this->m_face_sign[3] = true : this->m_face_sign[3] = false;
	this->m_xyz_id[2] == 1 ? this->m_face_sign[4] = true : this->m_face_sign[4] = false;
	this->m_xyz_id[2] == this->m_scale[2] ? this->m_face_sign[5] = true : this->m_face_sign[5] = false;
	
}

void Crowncube::set_face_id()
{
	//0 x_min
	this->m_face_id[0][0] = this->m_xyz_id[0] - 1;
	this->m_face_id[0][1] = this->m_xyz_id[1];
	this->m_face_id[0][2] = this->m_xyz_id[2];
	//1 x_max
	this->m_face_id[1][0] = this->m_xyz_id[0] + 1;
	this->m_face_id[1][1] = this->m_xyz_id[1];
	this->m_face_id[1][2] = this->m_xyz_id[2];
	//2 y_min
	this->m_face_id[2][0] = this->m_xyz_id[0];
	this->m_face_id[2][1] = this->m_xyz_id[1] - 1;
	this->m_face_id[2][2] = this->m_xyz_id[2];
	//3 y_max
	this->m_face_id[3][0] = this->m_xyz_id[0];
	this->m_face_id[3][1] = this->m_xyz_id[1] + 1;
	this->m_face_id[3][2] = this->m_xyz_id[2];
	//4 z_min
	this->m_face_id[4][0] = this->m_xyz_id[0];
	this->m_face_id[4][1] = this->m_xyz_id[1];
	this->m_face_id[4][2] = this->m_xyz_id[2] - 1;
	//5 z_max
	this->m_face_id[5][0] = this->m_xyz_id[0];
	this->m_face_id[5][1] = this->m_xyz_id[1];
	this->m_face_id[5][2] = this->m_xyz_id[2] + 1;
	
}

void Crowncube::set_face_center()
{
	//0 x_min
	this->m_face_center[0][0] = this->m_center[0] - this->m_size / 2.0;
	this->m_face_center[0][1] = this->m_center[1];
	this->m_face_center[0][2] = this->m_center[2];
	//1 x_max
	this->m_face_center[1][0] = this->m_center[0] + this->m_size / 2.0;
	this->m_face_center[1][1] = this->m_center[1];
	this->m_face_center[1][2] = this->m_center[2];
	//2 y_min
	this->m_face_center[2][0] = this->m_center[0];
	this->m_face_center[2][1] = this->m_center[1] - this->m_size / 2.0;
	this->m_face_center[2][2] = this->m_center[2];
	//3 y_max
	this->m_face_center[3][0] = this->m_center[0];
	this->m_face_center[3][1] = this->m_center[1] + this->m_size / 2.0;
	this->m_face_center[3][2] = this->m_center[2];
	//4 z_min
	this->m_face_center[4][0] = this->m_center[0];
	this->m_face_center[4][1] = this->m_center[1];
	this->m_face_center[4][2] = this->m_center[2] - this->m_size / 2.0;
	//5 z_max
	this->m_face_center[5][0] = this->m_center[0];
	this->m_face_center[5][1] = this->m_center[1];
	this->m_face_center[5][2] = this->m_center[2] + this->m_size / 2.0;
}

void Crowncube::set_face_sign(int _face_id_x, int _face_id_y, int _face_id_z)
{
	for (int i = 0; i < 6; i++)
	{
		if (this->face_id()[i][0] == _face_id_x && this->face_id()[i][1] == _face_id_y && this->face_id()[i][2] == _face_id_z)
		{
			this->m_face_sign[i] = true;
		}
	}
}



//方块面信息
bool* Crowncube::face_sign()
{
	return this->m_face_sign;
}

Eigen::Vector3i* Crowncube::face_id()
{
	return this->m_face_id;
}

Eigen::Vector3f* Crowncube::face_center()
{
	return this->m_face_center;
}

Eigen::MatrixXf Crowncube::face_point(int _face_id)
{
	Eigen::MatrixXf _vertices(4, 3);
	switch (_face_id)
	{
		//每个面对应点  从编号最小点起顺时针排序
		case 0:
			//0 x_min 0 1 3 2
			_vertices.row(0) = this->m_point_xyz[0].transpose();
			_vertices.row(1) = this->m_point_xyz[1].transpose();
			_vertices.row(2) = this->m_point_xyz[3].transpose();
			_vertices.row(3) = this->m_point_xyz[2].transpose();
			break;
			
		case 1:
			//1 x_max 4 5 7 6 
			_vertices.row(0) = this->m_point_xyz[4].transpose();
			_vertices.row(1) = this->m_point_xyz[5].transpose();
			_vertices.row(2) = this->m_point_xyz[7].transpose();
			_vertices.row(3) = this->m_point_xyz[6].transpose();
			break;

		case 2:
			//2 y_min 0 4 5 1
			_vertices.row(0) = this->m_point_xyz[0].transpose();
			_vertices.row(1) = this->m_point_xyz[4].transpose();
			_vertices.row(2) = this->m_point_xyz[5].transpose();
			_vertices.row(3) = this->m_point_xyz[1].transpose();
			break;

		case 3:
			//3 y_max 2 6 7 3
			_vertices.row(0) = this->m_point_xyz[2].transpose();
			_vertices.row(1) = this->m_point_xyz[6].transpose();
			_vertices.row(2) = this->m_point_xyz[7].transpose();
			_vertices.row(3) = this->m_point_xyz[3].transpose();
			break;

		case 4:
			//4 z_min 0 4 6 2
			_vertices.row(0) = this->m_point_xyz[0].transpose();
			_vertices.row(1) = this->m_point_xyz[4].transpose();
			_vertices.row(2) = this->m_point_xyz[6].transpose();
			_vertices.row(3) = this->m_point_xyz[2].transpose();
			break;

		case 5:
			//5 z_max 1 3 7 5
			_vertices.row(0) = this->m_point_xyz[1].transpose();
			_vertices.row(1) = this->m_point_xyz[3].transpose();
			_vertices.row(2) = this->m_point_xyz[7].transpose();
			_vertices.row(3) = this->m_point_xyz[5].transpose();
			break;

		default :
			printf("error,please input 0-5!");
	}
	return _vertices;
}

//方块点赋值
void Crowncube::set_point_xyz()
{
	//7(+,+,+)
	this->m_point_xyz[7][0] = this->m_center[0] + this->m_size / 2.0;
	this->m_point_xyz[7][1] = this->m_center[1] + this->m_size / 2.0;
	this->m_point_xyz[7][2] = this->m_center[2] + this->m_size / 2.0;
	//6(+,+,-)
	this->m_point_xyz[6][0] = this->m_center[0] + this->m_size / 2.0;
	this->m_point_xyz[6][1] = this->m_center[1] + this->m_size / 2.0;
	this->m_point_xyz[6][2] = this->m_center[2] - this->m_size / 2.0;
	//5(+,-,+)
	this->m_point_xyz[5][0] = this->m_center[0] + this->m_size / 2.0;
	this->m_point_xyz[5][1] = this->m_center[1] - this->m_size / 2.0;
	this->m_point_xyz[5][2] = this->m_center[2] + this->m_size / 2.0;
	//4(+,-,-)
	this->m_point_xyz[4][0] = this->m_center[0] + this->m_size / 2.0;
	this->m_point_xyz[4][1] = this->m_center[1] - this->m_size / 2.0;
	this->m_point_xyz[4][2] = this->m_center[2] - this->m_size / 2.0;
	//3(-,+,+)
	this->m_point_xyz[3][0] = this->m_center[0] - this->m_size / 2.0;
	this->m_point_xyz[3][1] = this->m_center[1] + this->m_size / 2.0;
	this->m_point_xyz[3][2] = this->m_center[2] + this->m_size / 2.0;
	//2(-,+,-)	
	this->m_point_xyz[2][0] = this->m_center[0] - this->m_size / 2.0;
	this->m_point_xyz[2][1] = this->m_center[1] + this->m_size / 2.0;
	this->m_point_xyz[2][2] = this->m_center[2] - this->m_size / 2.0;
	//1(-,-,+)
	this->m_point_xyz[1][0] = this->m_center[0] - this->m_size / 2.0;
	this->m_point_xyz[1][1] = this->m_center[1] - this->m_size / 2.0;
	this->m_point_xyz[1][2] = this->m_center[2] + this->m_size / 2.0;
	//0(-,-,-)
	this->m_point_xyz[0][0] = this->m_center[0] - this->m_size / 2.0;
	this->m_point_xyz[0][1] = this->m_center[1] - this->m_size / 2.0;
	this->m_point_xyz[0][2] = this->m_center[2] - this->m_size / 2.0;
}

//方块点信息
Eigen::Vector3f* Crowncube::point_xyz()
{
	return this->m_point_xyz;
}