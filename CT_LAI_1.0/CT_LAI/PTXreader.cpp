
#include "PTXReader.h"


bool PTXreader::open()
{
	if (this->filename == NULL)
	{
		printf("ERROR: File name not set\n");
		return false;
	}

	if (0 != _wfopen_s(&this->fp, filename, L"rb"))
	{
		printf("ERROR: Open file failed\n");
		return false;
	}
	return true;
}

void PTXreader::close()
{
	fclose(fp);
}

bool PTXreader::read_header()
{
	printf("TLS data (.ptx): Reading header...");
	clock_t launch = clock();

	if(0 != _fseeki64_nolock(this->fp, 0, SEEK_END) )
	{
		printf("Warning: seek end failed\n");
	}
	else
	{
		this->stream_pos_end_ = _ftelli64_nolock(fp);
	}
	rewind(this->fp);

	fscanf_s(this->fp, "%d\n", &n_cols_in_);
	this->n_cols_out_ = this->n_cols_in_;
	fscanf_s(this->fp, "%d\n", &n_rows_in_);
	this->n_rows_out_ = this->n_rows_in_;
	this->n_points_in_ = (long long)this->n_cols_in_ * this->n_rows_in_;

	if (0 == this->n_points_in_)
	{
		printf("ERROR: Point number = 0\n");
		return false;
	}

	fscanf_s(this->fp, "%lf %lf %lf\n", &scanner_position(0), &scanner_position(1), &scanner_position(2));

	fscanf_s(this->fp, "%lf %lf %lf\n", &scanner_x_axis(0), &scanner_x_axis(1), &scanner_x_axis(2));
	fscanf_s(this->fp, "%lf %lf %lf\n", &scanner_y_axis(0), &scanner_y_axis(1), &scanner_y_axis(2));
	fscanf_s(this->fp, "%lf %lf %lf\n", &scanner_z_axis(0), &scanner_z_axis(1), &scanner_z_axis(2));


	fscanf_s(this->fp, "%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n", \
		&rtMatrix(0, 0), &rtMatrix(0, 1), &rtMatrix(0, 2), &rtMatrix(0, 3), \
		&rtMatrix(1, 0), &rtMatrix(1, 1), &rtMatrix(1, 2), &rtMatrix(1, 3), \
		&rtMatrix(2, 0), &rtMatrix(2, 1), &rtMatrix(2, 2), &rtMatrix(2, 3), \
		&rtMatrix(3, 0), &rtMatrix(3, 1), &rtMatrix(3, 2), &rtMatrix(3, 3));

	clock_t done = clock();
	printf(" %.3lf s\n", double(done - launch) / CLOCKS_PER_SEC);
	return true;
}

void PTXreader::set_angle_extent_mask(angle_extent _angle_extent)
{
	this->angle_extent_mask_ = _angle_extent;
	this->has_angle_extent_ = true;

}

void PTXreader::set_sky_fillvalue(float _sky_fillvalue)
{
	this->sky_fillvalue = _sky_fillvalue;
}

void PTXreader::set_intensity_threshold(float _threshold)
{
	this->intensity_threshold = _threshold;
}

void PTXreader::set_resoultion_degree(float _resoultion_degree)
{
	this->resolution_degree = _resoultion_degree;
}

int PTXreader::read_data()
{


#pragma region subsample

	this->n_cols_out_ = (this->n_cols_in_ - 1) / this->cols_skip_ + 1;
	this->n_rows_out_ = (this->n_rows_in_ - 1) / this->rows_skip_ + 1;
	this->n_points_out_ = (long long)this->n_cols_out_ * this->n_rows_out_;

#pragma endregion subsample

	//std::cout << "time: " << (done - launch) / CLOCKS_PER_SEC << "s" << std::endl;

	// Pre-read data to obtain a angle map
#pragma region Pre-read a sample from the data

	printf("TLS data (.ptx): Pre-reading data...");

	clock_t launch = clock();

	// Pre-read a sampled data
	// number of samples
	int n_cols_sample = 180;   //每1°抽一样
	int n_rows_sample = 141;

	int cols_skip_sample = max(static_cast<int>(this->n_cols_in_ / n_cols_sample), 10);
	int rows_skip_sample = max(static_cast<int>(this->n_rows_in_ / n_rows_sample), 10);

	rows_skip_sample = rows_skip_sample / 8 * 8;

	n_cols_sample = (this->n_cols_in_ - 1) / cols_skip_sample + 1;
	n_rows_sample = (this->n_rows_in_ - 1) / rows_skip_sample + 1;

	this->xyz.resize(n_cols_sample* n_rows_sample, 3);


	long long stream_pos_start = _ftelli64_nolock(fp);
	Array<long long, Dynamic, 1> stream_pos_sample(n_cols_sample);	// data pos of each column for fread

	char s[255];
	int k = 0;
	int k_col = 0;
	for (int i = 0; i < this->n_cols_in_; ++i)
	{
		for (int j = 0; j < this->n_rows_in_; ++j)
		{
			if (i % cols_skip_sample == 0 && j % rows_skip_sample == 0)
			{
				if (j == 0)
				{
					stream_pos_sample(k_col++) = _ftelli64_nolock(fp);
					//printf_s( "%d\t%ld\n", k_col - 1, stream_pos_sample(k_col - 1));
				}

				fscanf_s(this->fp, "%f %f %f %*[^\n]\n", &xyz(k, 0), &xyz(k, 1), &xyz(k, 2));
				//fgets(s, 255, this->fp);
				++k;
			}
			else
			{
				//fscanf_s(this->fp, "%*[^\n]%*c");
				fgets(s, 255, this->fp);
			}

		}
	}




#pragma endregion Pre-read a sample from the data


#pragma region Calculate data range to read 
	// Calculate angle map of pre-read sample data 
	this->directions = (this->xyz * this->rtMatrix.block(0, 0, 3, 3).cast<float>());
	//预读采用scanner坐标系 (采用扫校轴坐标系)
	this->directions = this->xyz;
	this->zeniths = (this->directions.col(2).array() / this->directions.rowwise().norm().array()).acos();   //cos(zenith) = z
	this->azimuths = this->directions.col(1).cwiseQuotient(this->directions.col(0)).array().atan();   //tan(azimuth) = y/x
	this->azimuths = (this->directions.col(0).array() < 0).select(this->azimuths + M_PI, this->azimuths);
	this->azimuths = (this->directions.col(0).array() > 0 && this->directions.col(1).array() < 0).select(this->azimuths + 2 * M_PI, this->azimuths);
	Map<ArrayXXf> azimuth_image(this->azimuths.data(), n_rows_sample, n_cols_sample);
	Map<ArrayXXf> zenith_image(this->zeniths.data(), n_rows_sample, n_cols_sample);


	// Calculate four indices for angle extent


	// Calculate angle extent of PTX data
	ArrayXf min_azimuth_col = azimuth_image.isNaN().select(99, azimuth_image).colwise().minCoeff();
	ArrayXf max_azimuth_col = azimuth_image.isNaN().select(-99, azimuth_image).colwise().maxCoeff();
	ArrayXf min_zenith_row = zenith_image.isNaN().select(99, zenith_image).rowwise().minCoeff();
	ArrayXf max_zenith_row = zenith_image.isNaN().select(-99, zenith_image).rowwise().maxCoeff();


	this->angle_extent_data_.azimuth_min = min_azimuth_col.minCoeff();
	this->angle_extent_data_.azimuth_max = max_azimuth_col.maxCoeff();
	this->angle_extent_data_.zenith_min = min_zenith_row.minCoeff();
	this->angle_extent_data_.zenith_max = max_zenith_row.maxCoeff();
	
	if (!this->has_angle_extent_)
	{
		this->set_angle_extent_mask(angle_extent_data_);
	}
	
	if (this->angle_extent_data_.azimuth_min > this->angle_extent_mask_.azimuth_max || \
		this->angle_extent_data_.azimuth_max < this->angle_extent_mask_.azimuth_min || \
		this->angle_extent_data_.zenith_min > this->angle_extent_mask_.zenith_max || \
		this->angle_extent_data_.zenith_max < this->angle_extent_mask_.zenith_min )		// no intersection
	{
		return -1;
	}

	// Calculate index range in PTX data corresponding to envelope 
	int index_start_col, index_end_col, index_start_row, index_end_row;  // index of angle extent in raw data. [ index_start_* , index_end_* )
	int index_start_col_sample, index_end_col_sample, index_start_row_sample, index_end_row_sample; // index in sampled data
	int index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2; //temp index
	int index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2; //temp index


	(min_azimuth_col - this->angle_extent_mask_.azimuth_min).abs().minCoeff(&index_min_azimuth_1);
	(min_azimuth_col - this->angle_extent_mask_.azimuth_max).abs().minCoeff(&index_max_azimuth_1);
	(max_azimuth_col - this->angle_extent_mask_.azimuth_min).abs().minCoeff(&index_min_azimuth_2);
	(max_azimuth_col - this->angle_extent_mask_.azimuth_max).abs().minCoeff(&index_max_azimuth_2);

	(min_zenith_row - this->angle_extent_mask_.zenith_min).abs().minCoeff(&index_min_zenith_1);
	(min_zenith_row - this->angle_extent_mask_.zenith_max).abs().minCoeff(&index_max_zenith_1);
	(max_zenith_row - this->angle_extent_mask_.zenith_min).abs().minCoeff(&index_min_zenith_2);
	(max_zenith_row - this->angle_extent_mask_.zenith_max).abs().minCoeff(&index_max_zenith_2);


	int n_cols_data, n_rows_data; // number of columns and rows in angle extent

	// Row range
	index_start_row_sample = Array4i(index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2).minCoeff() - 1;
	index_end_row_sample = Array4i(index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2).maxCoeff() + 1;

	index_start_row = index_start_row_sample * rows_skip_sample;
	index_end_row = index_end_row_sample * rows_skip_sample - 1;
	if (index_start_row < 0) index_start_row = 0;
	if (index_end_row > this->n_rows_in_ - 1) index_end_row = this->n_rows_in_ - 1;

	n_rows_data = index_end_row - index_start_row;

	// Colume range
	bool is_ascending_azimuth = (min_azimuth_col.segment(1, n_cols_sample - 1) > min_azimuth_col.segment(0, n_cols_sample - 1)).count() > (n_cols_sample / 2);

	if (is_ascending_azimuth == (index_min_azimuth_1 < index_max_azimuth_1))		  //  Continuous azimuth
	{
		index_start_col_sample = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).minCoeff() - 1;
		index_end_col_sample = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).maxCoeff() + 1;
	}
	else																			// Splitted azimuth
	{
		
		index_start_col_sample = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).maxCoeff() - 1;
		index_end_col_sample = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).minCoeff() + 1;
		//printf("Data split\n");
		//getc(stdin);
	}

	//2017/09/22 Fix: index_start_col_sample != -1
	index_start_col_sample = max(index_start_col_sample, 0);
	index_end_col_sample = min(index_end_col_sample, n_cols_sample - 1);

	index_start_col = index_start_col_sample * cols_skip_sample;			
	index_end_col = index_end_col_sample * cols_skip_sample - 1;

	if (index_start_col_sample < index_end_col_sample)		//  Continuous azimuth
	{
		if (index_start_col < 0) index_start_col = 0;
		if (index_end_col > this->n_cols_in_ - 1) index_end_col = this->n_cols_in_ - 1;

		n_cols_data = index_end_col - index_start_col ;	
	}
	else														// Splitted azimuth
	{
		n_cols_data = this->n_cols_in_ - index_start_col + index_end_col;
	}
	

	//index_start_row = 0;
	//index_end_row = this->n_rows_in_ - 1;

	this->n_cols_out_ = (n_cols_data - 1) / this->cols_skip_ + 1;
	this->n_rows_out_ = (index_end_row ) / this->rows_skip_ - (index_start_row - 1) / this->rows_skip_;
	this->n_points_out_ = (long long)this->n_cols_out_ * this->n_rows_out_;


	clock_t done = clock();
	printf(" %.3lf s\n", double(done - launch) / CLOCKS_PER_SEC);

	printf("                 Extent: Azimuth = [%.2f, %.2f], Zenith = [%.2f, %.2f] (rad)\n", \
		this->angle_extent_data_.azimuth_min, \
		this->angle_extent_data_.azimuth_max, \
		this->angle_extent_data_.zenith_min, \
		this->angle_extent_data_.zenith_max);

#pragma endregion Calculate data range to read  


#pragma region Read_data


	printf("TLS data (.ptx): Reading data...");
	launch = clock();

	this->xyz.resize(this->n_points_out_, 3);
	this->intensity.resize(this->n_points_out_);
	this->rgb.resize(this->n_points_out_, 3);
	char * buffer = NULL;
	if (index_start_col_sample < index_end_col_sample)		//  Continuous azimuth
	{
		
		long long data_length = (stream_pos_sample(index_end_col_sample) - stream_pos_sample(index_start_col_sample)) / sizeof(char);
		buffer = new char[data_length];
		if (buffer == NULL)
		{
			printf("ERROR: Memory allocation failed\n");
			return false;
		}

		_fseeki64_nolock(fp, stream_pos_sample(index_start_col_sample), SEEK_SET);
		if (data_length != fread(buffer, sizeof(char), data_length, fp))
		{
			printf("ERROR: fread failed\n");
			return false;
		}
	}
	else   // Splitted azimuth
	{
		long long data_length_1 = (this->stream_pos_end_ - stream_pos_sample(index_start_col_sample)) / sizeof(char);
		long long data_length_2 = (stream_pos_sample(index_end_col_sample) - stream_pos_sample(0)) / sizeof(char);
		

		buffer = new char[data_length_1 + data_length_2];
		if (buffer == NULL)
		{
			printf("ERROR: Memory allocation failed\n");
			return false;
		}

		_fseeki64_nolock(fp, stream_pos_sample(index_start_col_sample), SEEK_SET);
		if (data_length_1 != fread(buffer, sizeof(char), data_length_1, fp))
		{
			printf("ERROR: fread 1 failed\n");
			return false;
		}


		_fseeki64_nolock(fp, stream_pos_sample(0), SEEK_SET);
		if (data_length_2 != fread(buffer + data_length_1, sizeof(char), data_length_2, fp))
		{
			printf("ERROR: fread 2 failed\n");
			return false;
		}
	}
	
	char * p;
	char * line;

	//done = clock();
	//printf(" %.3lf s\n", double(done - launch) / CLOCKS_PER_SEC);

	//launch = clock();

	line = strtok_s(buffer, "\n", &p);
	k = 0;
	for (int i = 0; i < n_cols_data; ++i)
	{
		for (int j = 0; j < n_rows_in_; ++j)
		{
			if (index_start_row <= j && j <= index_end_row && i % this->cols_skip_ == 0 && j % this->rows_skip_ == 0)
			{
				sscanf_s(line, "%f %f %f %f %hhu %hhu %hhu\n", &xyz(k, 0), &xyz(k, 1), &xyz(k, 2), &intensity(k), &rgb(k, 0), &rgb(k, 1), &rgb(k, 2));
				++k;
			}
			line = strtok_s(NULL, "\n", &p);
		}
	}

	delete[] buffer;

	

	this->norms = this->xyz.rowwise().norm();


	if (this->sky_fillvalue < this->intensity_threshold)
	{
		this->isgaps = (this->intensity < this->intensity_threshold);
	}
	else
	{
		this->isgaps = (this->intensity > this->intensity_threshold);
	}
	
	//使用(扫校轴坐标系)  扫描仪坐标系
	//this->directions = (this->xyz.cast<double>() * this->rtMatrix.block(0, 0, 3, 3)).cast<float>();
	//this->directions = (this->xyz.cast<double>() * this->rtMatrix.block(0, 0, 3, 3) + scanner_position.transpose()).cast<float>();
	this->directions = this->xyz;
	this->zeniths = (this->directions.col(2).array() / this->get_norms()).acos();   //cos(zenith) = z
	this->azimuths = this->directions.col(1).cwiseQuotient(this->directions.col(0)).array().atan();   //tan(azimuth) = y/x
	this->azimuths = (this->directions.col(0).array() < 0).select(this->azimuths + M_PI, this->azimuths);
	this->azimuths = (this->directions.col(0).array() > 0 && this->directions.col(1).array() < 0).select(this->azimuths + 2 * M_PI, this->azimuths);

	done = clock();
	printf(" %.3lf s\n", double(done - launch) / CLOCKS_PER_SEC);

#pragma endregion Read data

#ifdef _DEBUG
	Map<ArrayXXf> x_image(this->xyz.col(0).data(), n_rows_out_, n_cols_out_);
	Map<ArrayXXf> azimuth_image2(this->azimuths.data(), n_rows_out_, n_cols_out_);
	Map<ArrayXXf> zenith_image2(this->zeniths.data(), n_rows_out_, n_cols_out_);
	Map<ArrayXXf> intensity_image(this->intensity.data(), n_rows_out_, n_cols_out_);
	Map<ArrayXXb> gap_image(this->isgaps.data(), n_rows_out_, n_cols_out_);

	Matrix<uint8_t, Dynamic, Dynamic, RowMajor> rgb2 = this->rgb;
	
	cout << x_image(0) << azimuth_image2(0) << zenith_image2(0) << intensity_image(0) << rgb2(0) << endl;
	
#endif

	return 0;
}

ArrayXf PTXreader::zenith_azimuth_interpolation()
{
	//Hu 天顶角减小顺序    Trimble X7 天顶角增大顺序
	float espacement = (float)(this->resolution_degree / 180.0f * M_PI * this->rows_skip_);
	//float espacement = -(float)(this->resolution_degree / 180.0f * M_PI * this->rows_skip_);
	//size_t i = 0;
	
	this->zeniths_raw = (this->xyz.col(2).array() / this->get_norms()).acos();   //cos(zenith) = z
	this->azimuths_raw = this->xyz.col(1).cwiseQuotient(this->xyz.col(0)).array().atan();   //tan(azimuth) = y/x
	this->azimuths_raw = (this->xyz.col(0).array() < 0).select(this->azimuths_raw+M_PI, this->azimuths_raw);
	this->azimuths_raw = (this->xyz.col(0).array() > 0 && this->xyz.col(1).array() < 0).select(this->azimuths_raw + 2 * M_PI, this->azimuths_raw);
	
	//ArrayXb without_xyz = this->norms(j) < 0.00001;


	// Zenith azumuth interpolation
#pragma omp parallel for 
	for (Index i = 0; i < this->n_cols_out_; ++i)
	{
		bool isgaps = false;
		Index ij0 = i * this->n_rows_out_;
		Index istart = -1;
		Index n_gap = 0;
		float zenith1, zenith2;

		for (Index j = 0; j < this->n_rows_out_; ++j)
		{
			
			
			if (this->norms(ij0+j) < 0.00001)
			{

				if (isgaps)
				{
					++n_gap;
				}
				else
				{
					istart = ij0 + j;
					n_gap = 1;
					isgaps = true;

				}
				//xyz230425更新
				if (j == this->n_rows_out_ - 1)
				{
					//整列为间隙
					if (istart == ij0)
					{
						//赋前一列zenith值
						if (istart - n_gap > 0)
						{
							this->zeniths_raw.segment(istart, n_gap).setLinSpaced(n_gap, this->zeniths_raw(istart - n_gap), this->zeniths_raw(istart - 1));
							//赋前一列azimuth + espacement
							float azimuth_temp = 0.0;
							float azimuth_last = this->azimuths_raw(istart - 1);
							azimuth_temp = (azimuth_last < 0.01) ? 2 * M_PI : azimuth_last - espacement;  //方位角减小
							this->azimuths_raw.segment(istart, n_gap).setConstant(azimuth_temp);
						}
						//第一列
						else
						{
							this->zeniths_raw.segment(istart, n_gap).setLinSpaced(n_gap, this->angle_extent_mask_.zenith_min, this->angle_extent_mask_.zenith_max);
							this->azimuths_raw.segment(istart, n_gap).setConstant(this->angle_extent_mask_.azimuth_max);
						}

					}
					//该间隙点是数据最后一行，有间隙前段信息
					else
					{
						this->zeniths_raw.segment(istart - 1, n_gap + 1).setLinSpaced(n_gap + 1, this->zeniths_raw(istart - 1), this->zeniths_raw(istart - 1) + espacement * n_gap);
						this->azimuths_raw.segment(istart, n_gap).setConstant(this->azimuths_raw(istart - 1));
						//this->norms.segment(istart, n_gap).setConstant(9999);
					}
				}

			}
			else
			{
				if (isgaps)
				{
					//该间隙从0行开始，则只有间隙末端信息
					if (istart == ij0)
					{
						//printf("%f  %f  %f \n", espacement, (this->zeniths_raw(istart + n_gap) - espacement * n_gap), this->zeniths_raw(istart + n_gap));
						this->zeniths_raw.segment(istart, n_gap + 1).setLinSpaced(n_gap + 1, this->zeniths_raw(istart + n_gap) - espacement * n_gap, this->zeniths_raw(istart + n_gap));
						this->azimuths_raw.segment(istart, n_gap ).setConstant(this->azimuths_raw(istart + n_gap));
						//this->norms.segment(istart, n_gap).setConstant(9999);
					}
					//不是从0行开始的间隙，前后端信息皆有
					else
					{
						zenith1 = this->zeniths_raw(istart - 1);
						zenith2 = this->zeniths_raw(istart + n_gap);

						//if (zenith2 - zenith1 > 0)
						if (zenith1 - zenith2 > 0)  //xyz230424 Trimble X7
						{
							cout << "error: zenith1 < zenith2" << endl;
						}
						else
						{
							this->zeniths_raw.segment(istart - 1, n_gap + 2).setLinSpaced(n_gap + 2, zenith1, zenith2);
							this->azimuths_raw.segment(istart, n_gap).setConstant(this->azimuths_raw(istart - 1));
							//this->norms.segment(istart, n_gap).setConstant(9999);
						}

					}
					isgaps = false;

				}

			}
		}
	}

	ArrayXf norms_org = this->norms;
	Map<ArrayXXf> norms_org_image(norms_org.data(), this->n_rows_out_, this->n_cols_out_);
	Map<ArrayXXf> norms_image(this->norms.data(), this->n_rows_out_, this->n_cols_out_);
	Map<ArrayXXb> isgaps_image(this->isgaps.data(), this->n_rows_out_, this->n_cols_out_);
	
	//int demi_size = 2;
	//for (Index i = demi_size; i < this->n_rows_out_ - demi_size; ++i)
	//{
	//	Index ij0 = i * this->n_cols_out_;
	//	for (Index j = demi_size; j < this->n_cols_out_ - demi_size; ++j)
	//	{
	//		if (!isgaps_image(i,j) && norms_image(i,j) <  0.00001)
	//		{
	//			ArrayXXf block_norms = norms_org_image.block(i - demi_size, j - demi_size, 2 * demi_size, 2 * demi_size);
	//			float tmp = block_norms.sum() / (block_norms > 0.00001).count();
	//			norms_image(i, j) = tmp;
	//			//printf("(%d, %d): %f / %d = %f\n", i, j, block_norms.sum(), (block_norms > 0.00001).count(), tmp);
	//		}
	//	}
	//}


	// Norms interpolation
#pragma omp parallel for 
	for (int i = 0; i < this->n_rows_out_ ; ++i)
	{
		int ij0 = i * this->n_cols_out_;
		//#pragma omp parallel for 
		for (int j = 0; j < this->n_cols_out_ ; ++j)
		{
			if (norms_image(i, j) < 0.0001f)
			{
				if (isgaps_image(i, j))
				{
					norms_image(i, j) = 9999;
				}
				else
				{
					int demi_size = 2;
					int n_norms = 0;
					ArrayXXf block_norms;
					do
					{
						//printf("%d\t%d\t%d\t%d\n", max(i - demi_size, 0), max(j - demi_size, 0), min(2 * demi_size, this->n_rows_out_ - 1 - i + demi_size), min(2 * demi_size, this->n_cols_out_ - 1 - j + demi_size));
						block_norms = norms_org_image.block(max(i - demi_size, 0), max(j - demi_size, 0), \
							min(2 * demi_size, this->n_rows_out_ - 1 - max(i - demi_size, 0)), \
							min(2 * demi_size, this->n_cols_out_ - 1 - max(j - demi_size, 0)));		//Fix: 2017-12-12 rhhu 
						n_norms = (int)(block_norms > 0.0001f).count();
						if (demi_size > 100)		
						{
							
							break;
						}

						demi_size *= 2 ;

					} while (n_norms == 0);
					//if (block_norms.sum() < 0.0001f)
					//{
					//	printf("Warning \n");
					//}
					if (demi_size > 100)
					{
						norms_image(i, j) = 9999;
					}
					else
					{
						norms_image(i, j) = block_norms.sum() / n_norms;
					}
					
					//norms_image(i, j) = block_norms.maxCoeff();
					//printf("(%d, %d): %f / %d = %f\n", i, j, block_norms.sum(), n_norms, norms_image(i, j));
				}
			}
			
		}
	}
	

	//坐标反算
	this->xyz.col(0) = this->azimuths_raw.cos() * this->zeniths_raw.sin() * this->norms;
	this->xyz.col(1) = this->azimuths_raw.sin() * this->zeniths_raw.sin() * this->norms;
	this->xyz.col(2) = this->zeniths_raw.cos() * this->norms;

	//扫校轴坐标系
	this->directions = (this->xyz.cast<double>() * this->rtMatrix.block(0, 0, 3, 3)).cast<float>();
	this->zeniths = (this->directions.col(2).array() / this->get_norms()).acos();   //cos(zenith) = z
	this->azimuths = this->directions.col(1).cwiseQuotient(this->directions.col(0)).array().atan();   //tan(azimuth) = y/x
	this->azimuths = (this->directions.col(0).array() < 0).select(this->azimuths + M_PI, this->azimuths);
	this->azimuths = (this->directions.col(0).array() > 0 && this->directions.col(1).array() < 0).select(this->azimuths + 2 * M_PI, this->azimuths);


	this->is_zenith_azimuth_interporated = true;

	//ArrayXd azimuth2 = xyz.col(1).array().binaryExpr(xyz.col(0).array(), std::ptr_fun(atan2));

	return this->zeniths;
}

int PTXreader::n_cols()
{
	return this->n_cols_out_;
}

int PTXreader::n_rows()
{
	return this->n_rows_out_;
}

long long PTXreader::n_points()
{
	return this->n_points_out_;
}

long long PTXreader::n_points_data()
{
	return this->n_points_in_;
}

angle_extent PTXreader::get_angle_extent()
{
	return this->angle_extent_data_;
}

Vector3d PTXreader::get_scanner_position()
{
	return this->scanner_position;
}

Matrix4d PTXreader::get_rtMatrix()
{
	return this->rtMatrix;
}

MatrixXf PTXreader::get_directions()
{
	return this->directions;
}


ArrayXf PTXreader::get_norms()
{
	return this->norms;
}



ArrayXf PTXreader::get_zeniths()
{
	return this->zeniths;
}

ArrayXf PTXreader::get_intensity()
{
	return this->intensity;
}


ArrayXf PTXreader::get_azimuths()
{
	return this->azimuths;
}

ArrayXb PTXreader::get_gaps()
{
	return this->isgaps;
}



Map<ArrayXXf> PTXreader::get_norms_image()
{
	Map<ArrayXXf> _norms_image(this->norms.data(), this->n_rows_out_, this->n_cols_out_);
	return _norms_image;
}


Map<ArrayXXf> PTXreader::get_zeniths_image()
{
	Map<ArrayXXf> _zeniths_image(this->zeniths.data(), this->n_rows_out_, this->n_cols_out_);
	return _zeniths_image;
}


Map<ArrayXXf> PTXreader::get_azimuths_image()
{
	Map<ArrayXXf> _azimuths_image(this->azimuths.data(), this->n_rows_out_, this->n_cols_out_);
	return _azimuths_image;
}



Map<ArrayXXb> PTXreader::get_gaps_image()
{
	Map<ArrayXXb> _gaps_gaps(this->isgaps.data(), this->n_rows_out_, this->n_cols_out_);
	return _gaps_gaps;
}

Map<ArrayXXf> PTXreader::get_interpolated_zeniths_image()
{
	if (!is_zenith_azimuth_interporated)
	{
		this->zenith_azimuth_interpolation();
	}
	Map<ArrayXXf> _zeniths_image(this->zeniths.data(), this->n_rows_out_, this->n_cols_out_);
	return _zeniths_image;
}

Map<ArrayXXf> PTXreader::get_intensity_image()
{
	Map<ArrayXXf> _intensity_image(this->intensity.data(), this->n_rows_out_, this->n_cols_out_);
	return _intensity_image;
}




