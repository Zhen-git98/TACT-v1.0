#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // !1


#ifndef PTXREADER_H
#define PTXREADER_H


#include "commdef.h"


typedef Matrix< bool, Dynamic, 1> VectorXb;
typedef Matrix< bool, Dynamic, Dynamic> MatrixXb;
typedef Array< bool, Dynamic, 1> ArrayXb;
typedef Array< bool, Dynamic, Dynamic> ArrayXXb;



class PTXreader
{
	//公用操作
public:
	PTXreader() {};
	//PTXreader(const char* _filename):filename(_filename) {};
	PTXreader(const wchar_t* _filename, unsigned _cols_skip = 1, unsigned _rows_skip = 1) :filename(_filename), cols_skip_(_cols_skip), rows_skip_(_rows_skip) {};
	
	bool open();
	void close();
	bool read_header();
	void set_angle_extent_mask(angle_extent _angle_extent);
	void set_intensity_threshold(float _threshold);
	void set_sky_fillvalue(float _sky_fillvalue);
	void set_resoultion_degree(float _resoultion_degree);
	
	int read_data();

	//获取参数操作
public:
	int n_cols();
	int n_rows();
	long long  n_points();
	long long  n_points_data();
	//MatrixXd GetXYZ();
	angle_extent get_angle_extent();
	Vector3d get_scanner_position();
	Matrix4d get_rtMatrix();
	MatrixXf get_directions();
	ArrayXf get_norms();
	ArrayXf get_zeniths();
	ArrayXf get_azimuths();
	ArrayXb get_gaps();
	ArrayXf get_intensity();
	ArrayXf zenith_azimuth_interpolation();

	Map<ArrayXXf> get_norms_image();
	Map<ArrayXXf> get_zeniths_image();
	Map<ArrayXXf> get_azimuths_image();
	Map<ArrayXXb> get_gaps_image();
	Map<ArrayXXf> get_interpolated_zeniths_image();
	Map<ArrayXXf> get_intensity_image();

	//.PTX共有信息
private:
	const wchar_t* filename;
	FILE *fp;
	long long stream_pos_end_;
	float sky_fillvalue;
	float intensity_threshold;
	float resolution_degree = 360.0f / 10240.0f;
	int n_cols_in_;
	int n_rows_in_;
	long long n_points_in_;
	Matrix4d rtMatrix;
	Vector3d scanner_position;
	Vector3d scanner_x_axis;
	Vector3d scanner_y_axis;
	Vector3d scanner_z_axis;

	//获取参数操作
private:
	angle_extent angle_extent_mask_;
	angle_extent angle_extent_data_;
	bool has_angle_extent_ = false;
	int cols_skip_ ;  //实际获取数据时抽稀比例
	int rows_skip_ ;

	int n_cols_out_;
	int n_rows_out_;
	long long n_points_out_;

	


	MatrixXf xyz;   //coordinates in the scanner
	ArrayXf intensity;
	Matrix<uint8_t, Dynamic, Dynamic, RowMajor> rgb;
	ArrayXf zeniths_raw;	//zeniths in scanner
	ArrayXf azimuths_raw;	//azimuths in scanner
	ArrayXf norms;

	MatrixXf directions;	//transformed xyz coordinates in local system   xyz230414//from scanner (0,0,0) (scanner's orientation corrected)
	ArrayXf zeniths;		//transformed zeniths in local system                    //(scanner's orientation corrected)
	ArrayXf azimuths;		//transformed azimuths in local system                   //(scanner's orientation corrected)

	ArrayXb isgaps;
	
	bool is_zenith_azimuth_interporated = false;

};
#endif