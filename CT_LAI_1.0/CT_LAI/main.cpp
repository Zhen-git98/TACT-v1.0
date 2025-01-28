#include "commdef.h"
#include "Crowncube.h"
#include "PTXreader.h"
#include "GFun.h"
//#define Show

typedef pcl::PointXYZ PointT;	
typedef pcl::PointCloud<PointT> PointCloud;


//Create and initialise voxels based on the .pcd file
int InitCrowncube(const wstring _crown_path, int _block_scalexy, Crowncube** _block, Eigen::MatrixXf& _Box_vertices, bool _out_blockInfo, bool _show_viewer, int& _filled_block);

list<wstring> GetFileNames(wchar_t* lpPath, const wchar_t* ext);
//wstring to string
string wstring2string(wstring wstr);
//string to wstring
wstring string2wstring(string str);

void get_debug_img_path(char* img_path, const wchar_t* ptx_path, const wchar_t* envelope_path, const char* debug_dir, const char* append);

void write_debug_img(const char* img_path, Eigen::ArrayXXf& _image);
void write_debug_img(const char* img_path, Map<ArrayXXf>& _image);

struct angle_extent get_angle_extent(Eigen::MatrixXf _vertices);


bool IntersectTriangle(const Vector3f& orig, const Vector3f& dir,
    Vector3f& v0, Vector3f& v1, Vector3f& v2,
    float* t, float* u, float* v);

MatrixXi get_exit_face(Crowncube _block[], Eigen::Vector3f _scanner_position, int _block_num);

MatrixXf get_face_dir(angle_extent _face_extent, MatrixXf& _directions, ArrayXf& _min_azimuth_col, ArrayXf& _max_azimuth_col, ArrayXf& _min_zenith_row, ArrayXf& _max_zenith_row, int _image_rows, int _image_cols, face_inf& _face_inf);

//Caculate the path length
float get_paths(Vector2i _exit_face, Crowncube _block[], int _block_num, Eigen::Vector3f _scanner_position, Eigen::VectorXf& _path_ptx);

//slab-method
bool intersectWithAABB(Crowncube& _block, Eigen::Vector3f ori, Eigen::Vector3f dir, float& _path);

Eigen::VectorXf get_Gap(Crowncube _block[], MatrixXf& _exit_vertices, MatrixXf& _exit_dir, int _block_num, Eigen::Vector3f _scanner_position, int _bn, int _fn);

double* g_Gfun_values = 0;
int g_n_bins_gMes = -1;		
double g_angle_interval_G = 0.1;

int wmain(int argc, wchar_t* argv[])
{      
    //basic parameters
    wchar_t ptx_path[_MAX_PATH] = L"";
    wchar_t envelope_path[_MAX_PATH] = L"";

    wchar_t gMes_path[_MAX_PATH] = L"";
    double* gMes = 0;			

    list <wstring> fpaths_ptx, fpaths_envelope;
    int block_scalexy = 10;
    bool out_blockInfo = false;
    bool show_viewer = true;
    int g_n_skip = 1;  //.ptx submable
    float g_resolution_TLS = round(360.f / 17850.0f * 100) / 100;    //Trimble X7 CT DATA
    bool is_resolution_set = false;
    float sky_fillvalue = 0.0f;
    float intensity_threshold = 0.1f;
    bool is_intensity_threshold_set = false; //New: 20220907 by rhu, true: intensity_threshold is set by user, false: intensity_threshold is set by algorithm.

    bool img_debug = false;
    bool img_debug_block = false;
    


    if (argc == 1)
    {
        fprintf(stderr, "%ls is better run in the command line\n", argv[0]);

        fprintf(stderr, "enter input ptx file: "); fgetws(ptx_path, _MAX_PATH, stdin);

        fprintf(stderr, "enter input envelope_path file: "); fgetws(envelope_path, _MAX_PATH, stdin);

        //fprintf(stderr, "enter output file: "); fgetws(out_path, _MAX_PATH, stdin);
    }

    for (int i = 1; i < argc; i++)
    {
        if (argv[i][0] == '\0')
        {
            continue;
        }
        else if (wcscmp(argv[i], L"-img_debug") == 0)
        {
            img_debug = true;
        }
        else if (wcscmp(argv[i], L"-img_debug_block") == 0)
        {

            if ((i + 1) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            img_debug_block = true;

            ++i;
        }
        else if (wcscmp(argv[i], L"-ptx") == 0)
        {

            if ((i + 1) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            wcscpy_s(ptx_path, argv[i + 1]);

            ++i;
        }
        else if (wcscmp(argv[i], L"-envelope") == 0)
        {

            if ((i + 1) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            wcscpy_s(envelope_path, argv[i + 1]);
            ++i;
        }
        else if (wcscmp(argv[i], L"-g") == 0)  //hu: 2018-01-22
        {

            if ((i + 2) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            wcscpy_s(gMes_path, argv[i + 1]);
            g_n_bins_gMes = _wtoi(argv[i + 2]);
            i += 2;

            gMes = new double[g_n_bins_gMes];
            errno_t err;
            if (err = ReadMes(gMes_path, gMes, g_n_bins_gMes))
            {
                fprintf(stderr, "ERROR: Read %ls failed, error code '%d'\n", argv[i + 1], err);
            }

        }
        else if (wcscmp(argv[i], L"-intensity_threshold") == 0)
        {
            if ((i + 1) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            intensity_threshold = (float)_wtof(argv[i + 1]);
            is_intensity_threshold_set = true;
            ++i;
        }
        else if (wcscmp(argv[i], L"-xy_scale") == 0)
        {
            if ((i + 1) >= argc)
            {
                fprintf(stderr, "ERROR: '%ls' needs 1 argument: stop\n", argv[i]);
            }
            block_scalexy = (int) _wtof(argv[i + 1]);
            ++i;
        }
        else
        {
            fprintf(stderr, "ERROR: cannot understand argument '%ls'\n", argv[i]);
        }
    }

    //GMes
    //FILE* f_G180 = NULL;
    //if (NULL == (f_G180 = fopen("f_G36.txt", "a+")))
    //{
        //printf("Failed to creat Fout file!");
        //fclose(f_G180);
        //f_G180 = NULL;
    //}

    int panels_G = 1800;
    double max_angle_G = M_PI;
    double min_angle_G = 0.0;
    g_angle_interval_G = (max_angle_G - min_angle_G) / panels_G;

    if (g_n_bins_gMes > 0)
    {
        g_Gfun_values = new double[panels_G + 1];

#pragma omp parallel for 
        for (int i = 0; i <= panels_G; ++i)
        {
            double u1 = cos(max_angle_G + i * g_angle_interval_G);
            g_Gfun_values[i] = GFun(abs(u1), gMes, g_n_bins_gMes);
        }

        //G_value = GMean(g_Gfun_values, envelope_extent.zenith_min, envelope_extent.zenith_max, angleInterval);
    }

    //for (int Gi = 0; Gi <= panels_G; ++Gi)
    //{
        //fprintf(f_G180,"%lf\n", g_Gfun_values[Gi]);
    //}
    //fclose(f_G180);

    fpaths_envelope = GetFileNames(envelope_path, L"pcd");
    fpaths_ptx = GetFileNames(ptx_path, L"ptx");

    //string Fname = to_string(block_scalexy) + "-refine_outer-" + ".txt";
    string Fname = to_string(block_scalexy) + ".txt";
    FILE* f_out = NULL;
    if (NULL == (f_out = fopen(Fname.c_str(), "a+")))
    {
        printf("Failed to creat Fout file!");
        fclose(f_out);
        f_out = NULL;
    }

    for (list<wstring>::iterator it_path_env = fpaths_envelope.begin(); it_path_env != fpaths_envelope.end(); it_path_env++)
    {
        int MAX_FAVD_num = 0;
        
        vtkObject::GlobalWarningDisplayOff();
        
        Crowncube* block = NULL;
        int filled_block = 0;
        MatrixXf Box_vertices(8,3); 
        
        int block_num = InitCrowncube((*it_path_env).c_str(), block_scalexy, &block, Box_vertices, out_blockInfo, show_viewer, filled_block);

        
        for (list<wstring>::iterator it_path_ptx = fpaths_ptx.begin(); it_path_ptx != fpaths_ptx.end(); it_path_ptx++)
        {
            printf("Input     (envelope): %ls\n", (*it_path_env).c_str());
            printf("          (.ptx): %ls\n", (*it_path_ptx).c_str());
            const wchar_t* _ptx_path = (*it_path_ptx).c_str();
            const wchar_t* _envelope_path = (*it_path_env).c_str();


            //CalcFromData((*it_path_ptx).c_str(), (*it_path_env).c_str(), append, refine_envelope, intensity_threshold, zenith_low, zenith_high, zenith_increment);
           
#pragma region out_file
#pragma endregion out_file

#pragma region Read point cloud and envelope data
            
            //head information
            PTXreader ray((*it_path_ptx).c_str(), g_n_skip, g_n_skip);
            if (!ray.open()) return -1;
            if (!ray.read_header()) return -1;
            if (!is_resolution_set)
            {
                
                g_resolution_TLS = round(360.f / ray.n_cols() * 100) / 100;
                //g_resolution_TLS = 360.0f / 10240.f / round(ray.n_cols() / 10240.0f);
            }
            ray.set_resoultion_degree(g_resolution_TLS);
            ray.set_sky_fillvalue(sky_fillvalue);
            ray.set_intensity_threshold(intensity_threshold);
            Vector3f scanner_position = ray.get_scanner_position().cast<float>();
            Matrix4f rtMatrix = ray.get_rtMatrix().cast<float>();

            
            MatrixXf Box_vertices_ = Box_vertices;
            Box_vertices_.rowwise() -= scanner_position.transpose();
            Box_vertices_ *= rtMatrix.block(0, 0, 3, 3).inverse();
            angle_extent Box_extent = get_angle_extent(Box_vertices_);
            ray.set_angle_extent_mask(Box_extent);
            if (ray.read_data() == -1)
            {
                //fprintf(file_out, "No intersection\n\n");
                printf("No intersection\n\n");
                ray.close();
                //fclose(file_out);
                return 2;
            }
            ray.close();
          
            if (img_debug)
            {
               
                ArrayXXf zeniths = ray.get_zeniths_image();
                ArrayXXf azimuths = ray.get_azimuths_image();
                ArrayXXf intensity_image = ray.get_intensity_image();
                char debug_dir[_MAX_DIR] = "debug";

                ArrayXXf norms_image = ray.get_norms_image();
                //char debug_dir[_MAX_DIR] = "debug";
                _mkdir(debug_dir);

                char img_path_zenith[_MAX_PATH];
                get_debug_img_path(img_path_zenith, _ptx_path, _envelope_path, debug_dir, "_zenith");
                write_debug_img(img_path_zenith, zeniths);

                char img_path_azimuth[_MAX_PATH];
                get_debug_img_path(img_path_azimuth, _ptx_path, _envelope_path, debug_dir, "_azimuth");
                write_debug_img(img_path_azimuth, azimuths);

                char img_path_intensity[_MAX_PATH];
                get_debug_img_path(img_path_intensity, _ptx_path, _envelope_path, debug_dir, "_intensity");
                write_debug_img(img_path_intensity, intensity_image);

                char img_path_norms[_MAX_PATH];
                get_debug_img_path(img_path_norms, _ptx_path, _envelope_path, debug_dir, "_norms");
                write_debug_img(img_path_norms, norms_image);
            }

            
            // Zeniths interpolation
            Map<ArrayXXf> zeniths_interpolated_image = ray.get_interpolated_zeniths_image();
            //Map<ArrayXXf> zeniths_interpolated_image = ray.get_zeniths_image();
            Map<ArrayXXf> azimuth_interpolated_image = ray.get_azimuths_image();
            //MatrixXf directions_interpolated_image = ray.get_directions();
            ArrayXXf norms_image2 = ray.get_norms_image();

            if (img_debug)
            {
                char debug_dir[_MAX_DIR] = "debug";

                char img_path_zenith_interpolated[_MAX_PATH];
                get_debug_img_path(img_path_zenith_interpolated, _ptx_path, _envelope_path, debug_dir, "_zenith_interpolated");
                write_debug_img(img_path_zenith_interpolated, zeniths_interpolated_image);

                char img_path_azimuth_interpolated[_MAX_PATH];
                get_debug_img_path(img_path_azimuth_interpolated, _ptx_path, _envelope_path, debug_dir, "_azimuth_interpolated");
                write_debug_img(img_path_azimuth_interpolated, azimuth_interpolated_image);

            }

            Map<ArrayXXf> zenith_image = ray.get_zeniths_image();
            Map<ArrayXXf> azimuth_image = ray.get_azimuths_image();
            MatrixXf directions = ray.get_directions();
            ArrayXf min_azimuth_col = azimuth_image.colwise().minCoeff();
            ArrayXf max_azimuth_col = azimuth_image.colwise().maxCoeff();
            ArrayXf min_zenith_row = zenith_image.rowwise().minCoeff();
            ArrayXf max_zenith_row = zenith_image.rowwise().maxCoeff();
           
            //exit_face
            MatrixXi exit_face = get_exit_face(block, scanner_position, block_num);
            MatrixXf path_ptx(exit_face.rows(), block_num);
            
            for (int ef = 0; ef < exit_face.rows(); ef++)
            {
                int ef1 = exit_face(ef, 0);
                int ef2 = exit_face(ef, 1);
                MatrixXf face_vertices = block[ef1].face_point(ef2).rowwise() - scanner_position.transpose();
                angle_extent face_extent = get_angle_extent(face_vertices);
                face_inf finf;
             
                
                MatrixXf face_dir= get_face_dir(face_extent, directions, min_azimuth_col, max_azimuth_col, min_zenith_row, max_zenith_row, ray.n_rows(), ray.n_cols(), finf);
                if (img_debug_block)
                {
                    ArrayXf intensity = ray.get_intensity();
                    ArrayXf face_int(finf.face_cols * finf.face_rows);
                    int cnt = 0;
                    for (int col = finf.start_col; col <= finf.end_col; col++)
                    {
                        for (int row = finf.start_row; row <= finf.end_row; row++)
                        {
                            face_int(cnt) = intensity(col * ray.n_rows() + row);
                            cnt++;
                        }
                    }
                    Map<ArrayXXf> _face_int(face_int.data(), finf.face_rows, finf.face_cols);
                    char debug_dir[_MAX_DIR] = "debug";
                    string append_temp = "_intensity";
                    append_temp.append(to_string(ef1));
                    append_temp.append(".");
                    append_temp.append(to_string(ef2));
                    char img_path_face_intensity[_MAX_PATH];
                    get_debug_img_path(img_path_face_intensity, _ptx_path, _envelope_path, debug_dir, append_temp.c_str());
                    write_debug_img(img_path_face_intensity, _face_int);
                }
                   

                //calculate gap probability using enter_num as the weighting
                VectorXf Gap_Weight(2);
                Gap_Weight = get_Gap(block, face_vertices, face_dir, block_num, scanner_position, ef1, ef2);
                //printf("%f \n", Gap_face);
                
                //path_ptx: path length of each voxels along the path; rows: number of exit_faces，cols: number of non_empty voxels
                VectorXf _path_ptx(block_num);  //empty -1.0 
                float Path_len = get_paths(exit_face.row(ef), block, block_num, scanner_position, _path_ptx);  //total path length
                //printf("Path_len:%f  %lf\n", Path_len, block[1].size());

                //G
                double G_value = 0.5;
                if (g_n_bins_gMes > 0)
                    G_value = GMean(g_Gfun_values, face_extent.zenith_min, face_extent.zenith_max, g_angle_interval_G);
                //printf("G= %lf\n", G_value);

                
                double effFAVD = abs(-log(Gap_Weight(0)) / G_value / Path_len);
                if (effFAVD < 10.0 && Gap_Weight(1)>30)    //xyz0818 
                {
                                        
                    for (int i = 0; i < block_num; i++)
                    {
                        if (_path_ptx(i) > 0.01)
                        {
                            block[i].add_FAVD_W(effFAVD * Gap_Weight(1));
                            block[i].add_Weight_T(Gap_Weight(1) * 1.0);
                        }
                    }
                }
                //effLAI = min(effLAI, 10.0);
                //printf("effLAI= %lf\n", effLAI);

            }


        }

        //printf("MAX_FAVD_num is: %d  %d\n", MAX_FAVD_num, block[0].scale()[2]);
        printf("\n\nvoxel_size is %lf m\n", block[20].size());
        double PA = 0.0;
        for (int i = 0; i < block_num; i++)
        {            
           PA += block[i].FAVD() * pow(block[i].size(), 3.0);     
        }
        printf("Plant area of this tree crown is: %lf square meters\n",PA);

        fprintf(f_out, "x_scale, y_scale, z_scale, PA (sq m),block_size (m)\n");
        fprintf(f_out, " %d %d %d %lf %lf\n\n\n", block[0].scale()[0], block[0].scale()[1], block[0].scale()[2], PA, block[1].size());
        fprintf(f_out, "3D distribution of PAVD:\n");
        //Layered along the pulse distance direction, output from near to far.
        fprintf(f_out,"1. Layered along the pulse distance direction, output from near to far:\n");
        for (int y = 0; y < block[0].scale()[1]; y++)
        {
            for (int z = block[0].scale()[2] - 1; z >= 0; z--)
            {
                for (int x = 0; x < block[0].scale()[0]; x++)
                {
                    fprintf(f_out, "%lf ", block[x + y * block[0].scale()[0] + z * block[0].scale()[0] * block[0].scale()[1]].FAVD());
                }
                fprintf(f_out, "\n");
            }
            fprintf(f_out, "\n");
        }


        
        //Layered along the tree canopy height direction, output from the top to the bottom.
        fprintf(f_out,"2. Layered along the tree canopy height direction, output from the top to the bottom:\n");
        for (int z = block[0].scale()[2] - 1; z >= 0; z--)
        {
            for (int y = 0; y < block[0].scale()[1]; y++)
            {
                for (int x = 0; x < block[0].scale()[0]; x++)
                {
                    fprintf(f_out, "%lf ", block[x + y * block[0].scale()[0] + z * block[0].scale()[0] * block[0].scale()[1]].FAVD());
                }
                fprintf(f_out, "\n");
            }
            fprintf(f_out, "\n");
        }


        //3D result
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(255, 255, 255);
        viewer->addCoordinateSystem(3.0);
        //viewer->initCameraParameters();
        
        viewer->setCameraPosition(0, 0, 0, 0, 1, 0, 0, 0, 1, 0);

        // 颜色属性
        float min_FAVD = 99.0;
        float max_FAVD = -99.0;
        for (int ib = 0; ib < block_num; ib++)
        {
            if (!block[ib].is_empty())
            {
                if (block[ib].FAVD() > max_FAVD)
                    max_FAVD = block[ib].FAVD();
                if (block[ib].FAVD() < min_FAVD)
                    min_FAVD = block[ib].FAVD();
            }
        }

        float normal_FAVD = 0.0;
        int cID = 0;
        int r_, g_, b_ = 0;
        double r, g, b = 0.0;
        for (int jb = 0; jb < block_num; jb++)
        {
            if (!block[jb].is_empty())
            {
                //colour
                normal_FAVD = (block[jb].FAVD() - min_FAVD) / (max_FAVD - min_FAVD);
                cID = ceil(normal_FAVD * 10);  
                
                switch (cID)
                {
                
                case 1: r_ = 238; g_ = 247; b_ = 242; break;
                case 2: r_ = 228; g_ = 223; b_ = 215; break;
                case 3: r_ = 223; g_ = 236; b_ = 213; break;
                case 4: r_ = 198; g_ = 223; b_ = 200; break;
                case 5: r_ = 158; g_ = 204; b_ = 171; break;
                case 6: r_ = 104; g_ = 184; b_ = 142; break;
                case 7: r_ = 32; g_ = 161; b_ = 98; break;
                case 8: r_ = 36; g_ = 128; b_ = 103; break;
                case 9: r_ = 26; g_ = 104; b_ = 64; break;
                default: r_ = 37; g_ = 61; b_ = 36;

                }
                
                r = r_ * 1.0 / 255;
                g = g_ * 1.0 / 255;
                b = b_ * 1.0 / 255;
                //
                string name = "block" + to_string(jb);
                viewer->addCube
                   (block[jb].xyz_min()[0], block[jb].xyz_max()[0],
                    block[jb].xyz_min()[1], block[jb].xyz_max()[1],
                    block[jb].xyz_min()[2], block[jb].xyz_max()[2],
                    r, g, b, name.c_str());

                //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, name.c_str());  //颜色
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, name.c_str());   //渲染  FLAT GOURAUD PHONG
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, name.c_str());   //透明度
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name.c_str());  //POINTS  WIREFRAME  SURFACE
            }
        }

        //viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0.14, 0.3, 0.4, 0);

        // viewer
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }



        delete[] g_Gfun_values;
        g_Gfun_values = 0;
        delete[] gMes;
        gMes = 0;

        
        //delete
        for (int i = 0; i < block_num; i++)
            block[i].~Crowncube();
        delete block;
    }

    fclose(f_out);
    
    return 0;
}



int InitCrowncube(const wstring _crown_path, int _block_scalexy, Crowncube** _block, Eigen::MatrixXf& _Box_vertices, bool _out_blockInfo, bool _show_viewer, int& _filled_block)
{
#ifdef Show
    //创建视窗
    pcl::visualization::PCLVisualizer _viewer(wstring2string(_crown_path).c_str());
    _viewer.setBackgroundColor(1, 1, 1);
    _viewer.addCoordinateSystem(2);
#endif
    
    //read pcd (envelope)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(wstring2string(_crown_path), *cloud) == -1) {
        std::cerr << "Cannot read the point cloud file" << std::endl;
        return -1;
    }

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float width = max_pt[0] - min_pt[0];   //x
    float height = max_pt[1] - min_pt[1];  //y
    float depth = max_pt[2] - min_pt[2];   //z

    //AAB
    float box_sizexy = std::max(width, height);
    float block_size = box_sizexy / _block_scalexy;

    if (width > height)
    {
        max_pt[1] += (box_sizexy - height) / 2.0;
        min_pt[1] -= (box_sizexy - height) / 2.0;
    }

    else
    {
        max_pt[0] += (box_sizexy - width) / 2.0;
        min_pt[0] -= (box_sizexy - width) / 2.0;
    }

    //z
    int _block_scalez = ceil(depth / block_size);
    float box_sizez = _block_scalez * block_size;
    max_pt[2] += (box_sizez - depth) / 2.0;
    min_pt[2] -= (box_sizez - depth) / 2.0;

    //center
    Eigen::Vector3f box_center((max_pt[0] + min_pt[0]) / 2,
                               (max_pt[1] + min_pt[1]) / 2,
                               (max_pt[2] + min_pt[2]) / 2);

    _Box_vertices << max_pt[0], max_pt[1], max_pt[2], 
                     max_pt[0], max_pt[1], min_pt[2], 
                     max_pt[0], min_pt[1], max_pt[2], 
                     max_pt[0], min_pt[1], min_pt[2], 
                     min_pt[0], max_pt[1], max_pt[2], 
                     min_pt[0], max_pt[1], min_pt[2], 
                     min_pt[0], min_pt[1], max_pt[2], 
                     min_pt[0], min_pt[1], min_pt[2];


    // 初始化分块信息
    int block_num = _block_scalexy * _block_scalexy * _block_scalez;
    
    //动态分配block
    * _block = reinterpret_cast<Crowncube*>(::operator new(sizeof(Crowncube) * block_num));
    for (int i = 0; i < block_num; i++)
    {
        new(&(*_block)[i]) Crowncube(i);
    }

    int cnt = 0;
    for (int z_num = 0; z_num < _block_scalez; z_num++)
    {
        for (int y_num = 0; y_num < _block_scalexy; y_num++)
        {
            for (int x_num = 0; x_num < _block_scalexy; x_num++)
            {
                (*_block)[cnt].set_scale(_block_scalexy, _block_scalexy, _block_scalez);
                (*_block)[cnt].set_xyz_range(min_pt[0] + x_num * block_size,
                                             min_pt[1] + y_num * block_size,
                                             min_pt[2] + z_num * block_size, block_size);
                (*_block)[cnt].set_center();
                (*_block)[cnt].set_xyz_id(x_num + 1, y_num + 1, z_num + 1);
                (*_block)[cnt].set_is_outer(false);
                (*_block)[cnt].set_cube_face();
                (*_block)[cnt].set_face_id();
                (*_block)[cnt].set_size(block_size);
                (*_block)[cnt].set_FAVD(0.0);
                (*_block)[cnt].set_scancnt(0);
                (*_block)[cnt].set_face_center();
                (*_block)[cnt].set_point_xyz();
                (*_block)[cnt].set_FAVD_W(0.0);
                (*_block)[cnt].set_Weight_T(0.0);
                cnt++;
            }
        }
    }

    // 判断分块是否空块
    int filled_block = 0;
    int max_point = 0;
    for (int i = 0; i < block_num; i++)
    {
    
        //xyz20230818  有一定数量点才是非空，避免边缘较大体积空块,避免噪声点
        int num_point = 0;
        for (int j = 0; j < cloud->points.size(); j++)
        {
            if (cloud->points[j].x < (*_block)[i].xyz_min()[0] || cloud->points[j].x >(*_block)[i].xyz_max()[0] ||
                cloud->points[j].y < (*_block)[i].xyz_min()[1] || cloud->points[j].y >(*_block)[i].xyz_max()[1] ||
                cloud->points[j].z < (*_block)[i].xyz_min()[2] || cloud->points[j].z >(*_block)[i].xyz_max()[2])
            {
                continue;
            }
            else
            {
                num_point++;
            }
        }
        (*_block)[i].set_Pnum(num_point);
        if (max_point < num_point)
        {
            max_point = num_point;
        }
    }

    for (int i = 0; i < block_num; i++)
    {
        (*_block)[i].set_is_empty(true);
        (*_block)[i].set_rPnum(1.0 * (*_block)[i].Pnum() / max_point);

        if ((*_block)[i].xyz_max()[0]<min_pt[0] || (*_block)[i].xyz_min()[0]>max_pt[0] ||
            (*_block)[i].xyz_max()[1]<min_pt[1] || (*_block)[i].xyz_min()[1]>max_pt[1] ||
            (*_block)[i].xyz_max()[2]<min_pt[2] || (*_block)[i].xyz_min()[2]>max_pt[2])
        {
            continue;
        }

        if ((*_block)[i].Pnum() > max_point * 0.03)
        {
            (*_block)[i].set_is_empty(false);
            filled_block++;
        }
    }

    printf("Non_empty voxels = %d\n\n\n", filled_block);

    // 计算非空分块面标识
    for (int i = 0; i < block_num; i++)
    {
        //只对空块进行处理
        if (!(*_block)[i].is_empty())
        {
            continue;
        }
        
        //先找到空块的非外向面
        for (int j = 0; j < 6; j++)
        {
            if (!(*_block)[i].face_sign()[j])
            {
                //分块面id 等于 block的xyz_id 在对应维度（x，y，z）方向上+-1
                //共面id等同于相邻块xyz_id   分块编码求分块序号公式  x + y * scale_x + z * scalex * scaley  
                int temp_ID = ((*_block)[i].face_id()[j][0] - 1) + ((*_block)[i].face_id()[j][1] - 1) * _block_scalexy + ((*_block)[i].face_id()[j][2] - 1) * _block_scalexy * _block_scalexy;
                if (!(*_block)[temp_ID].is_empty()) { (*_block)[temp_ID].set_is_outer(true); }
                //将其相邻块的共面设为外向面
                (*_block)[temp_ID].set_face_sign((*_block)[i].xyz_id()[0], (*_block)[i].xyz_id()[1], (*_block)[i].xyz_id()[2]);
            }
        }
    }

    // 输出分块信息
    if (_out_blockInfo)
    {
        FILE* block_info = NULL;
        if (NULL == (block_info = fopen("block_info.txt", "a+")))
        {
            printf("Failed to creat block_info file!");
            fclose(block_info);
            block_info = NULL;
        }

        fprintf(block_info, "scale    num   size\n");
        fprintf(block_info, "%d %d %lf\n", _block_scalexy, block_num, block_size);
        fprintf(block_info, "x_min   x_max   y_min  y_max   z_min   z_max\n");
        for (int i = 0; i < block_num; i++)
        {
            //if (!(*_block)[i].is_empty())
            //{
                fprintf(block_info, "%d %d %d %d %d %d %d %d %d %d\n", 
                    (*_block)[i].ID(),(*_block)[i].xyz_id()[0], (*_block)[i].xyz_id()[1], (*_block)[i].xyz_id()[2],
                    (*_block)[i].face_sign()[0], (*_block)[i].face_sign()[1], (*_block)[i].face_sign()[2],
                    (*_block)[i].face_sign()[3], (*_block)[i].face_sign()[4], (*_block)[i].face_sign()[5]);
            //}
      
        }

        fclose(block_info);
        block_info = NULL;
    }

#ifdef Show
    // 绘制
    if (_show_viewer)
    {
        // 载入并设置crown属性
        _viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
        _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "cloud");
        _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");


        //载入并设置box属性
        /*
        pcl::PointXYZ center_pt(box_center(0), box_center(1), box_center(2));
        (*_viewer).addCube(box_center[0] - box_size / 2, box_center[0] + box_size / 2, box_center[1] - box_size / 2, box_center[1] + box_size / 2, box_center[2] - box_size / 2, box_center[2] + box_size / 2, 1.0, 1.0, 1.0, "Crowncube");
        (*_viewer).setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "Crowncube");
        (*_viewer).setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "Crowncube");
        */

        //显示block
        for (int j = 0; j < block_num; j++)
        {
            if (!(*_block)[j].is_empty())
            {
                string name = "block" + to_string(j);
                              _viewer.addCube((*_block)[j].center()[0] - block_size / 2.0,
                                  (*_block)[j].center()[0] + block_size / 2.0,
                                  (*_block)[j].center()[1] - block_size / 2.0,
                                  (*_block)[j].center()[1] + block_size / 2.0,
                                  (*_block)[j].center()[2] - block_size / 2.0,
                                  (*_block)[j].center()[2] + block_size / 2.0,
                              1.0, 1.0, 1.0, name.c_str());
                _viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, name.c_str());
                _viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name.c_str());
                _viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1, name.c_str());
            }
        }
    }
    
    while (!_viewer.wasStopped()) {
        _viewer.spinOnce();
    }
#endif 


    return block_num;
}

string wstring2string(wstring wstr)
{
    string result;
    int len = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), NULL, 0, NULL, NULL);
    if (len <= 0)return result;
    char* buffer = new char[len + 1];
    if (buffer == NULL)return result;
    WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), buffer, len, NULL, NULL);
    buffer[len] = '\0';
    result.append(buffer);
    delete[] buffer;
    return result;
}

wstring string2wstring(string str)
{
    wstring result;
    int len = MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.size(), NULL, 0);
    if (len < 0)return result;
    wchar_t* buffer = new wchar_t[len + 1];
    if (buffer == NULL)return result;
    MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.size(), buffer, len);
    buffer[len] = '\0';
    result.append(buffer);
    delete[] buffer;
    return result;
}

list<wstring> GetFileNames(wchar_t* lpPath, const wchar_t* ext)
{
    list<wstring> fPaths;
    wchar_t file_path[_MAX_PATH];

    wchar_t drive[_MAX_DRIVE];
    wchar_t dir[_MAX_DIR];
    wchar_t fname[_MAX_FNAME];
    //wchar_t ext[_MAX_EXT];
    _wsplitpath(lpPath, drive, dir, fname, NULL);


    wchar_t szFind[_MAX_PATH];
    WIN32_FIND_DATA FindFileData;
    wcscpy_s(szFind, _MAX_PATH, lpPath);
    HANDLE hFind = FindFirstFile((LPCTSTR)szFind, &FindFileData);
    if (INVALID_HANDLE_VALUE == hFind)    return fPaths;


    if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)//如果是目录
    {
        if (FindFileData.cFileName[0] != '.')//排除.和..文件夹
        {
            wcscat_s(szFind, _MAX_PATH, L"\\*.");
            wcscat_s(szFind, _MAX_PATH, ext);
            hFind = FindFirstFile((LPCTSTR)szFind, &FindFileData);
            if (INVALID_HANDLE_VALUE == hFind)    return fPaths;
        }
    }
    else
    {
        _wmakepath_s(lpPath, _MAX_PATH, drive, dir, fname, ext);
    }


    while (TRUE)
    {
        if (FindFileData.cFileName[0] != '.')
        {
            _wmakepath_s(file_path, _MAX_PATH, drive, dir, FindFileData.cFileName, NULL);
            fPaths.push_back(file_path);
        }
        if (!FindNextFile(hFind, &FindFileData))    break;

    }
    FindClose(hFind);
    return fPaths;
}

void get_debug_img_path(char* img_path, const wchar_t* ptx_path, const wchar_t* envelope_path, const char* debug_dir, const char* append)
{
    wchar_t fname[_MAX_FNAME];
    wchar_t fname_append[_MAX_FNAME];

    //printf("%ls\n", ptx_path);
    //printf("%ls\n", envelope_path);
    _wsplitpath_s(ptx_path, NULL, 0, NULL, 0, fname, _MAX_FNAME, NULL, 0);
    _wsplitpath_s(envelope_path, NULL, 0, NULL, 0, fname_append, _MAX_FNAME, NULL, 0);
    sprintf(img_path, "%s\\%ws_%ws%s.%s", debug_dir, fname, fname_append, append, "tif");
}

void write_debug_img(const char* img_path, Map<ArrayXXf>& _image)
{
    FIBITMAP* dib = FreeImage_AllocateT(FIT_FLOAT, static_cast<int>(_image.rows()), static_cast<int>(_image.cols()), 32);
    BYTE* bits = FreeImage_GetBits(dib);
    memcpy(bits, (BYTE*)_image.data(), sizeof(_image(0, 0)) * _image.size());
    bool state = FreeImage_Save(FIF_TIFF, dib, img_path, TIFF_DEFAULT);
    FreeImage_Unload(dib);
}

void write_debug_img(const char* img_path, Eigen::ArrayXXf& _image)
{
    FIBITMAP* dib = FreeImage_AllocateT(FIT_FLOAT, static_cast<int>(_image.rows()), static_cast<int>(_image.cols()), 32);
    BYTE* bits = FreeImage_GetBits(dib);
    memcpy(bits, (BYTE*)_image.data(), sizeof(_image(0, 0)) * _image.size());
    bool state = FreeImage_Save(FIF_TIFF, dib, img_path, TIFF_DEFAULT);
    FreeImage_Unload(dib);
}

struct angle_extent get_angle_extent(Eigen::MatrixXf _vertices)
{
    ArrayXf zeniths = _vertices.col(2).cwiseQuotient(_vertices.rowwise().norm()).array().cast<float>().acos();   //cos(zenith) = z
    ArrayXf azimuths = _vertices.col(1).cwiseQuotient(_vertices.col(0)).array().cast<float>().atan();   //tan(azimuth) = y/x
    azimuths = (_vertices.col(0).array() < 0).select(azimuths + M_PI, azimuths);
    azimuths = (_vertices.col(0).array() > 0 && _vertices.col(1).array() < 0).select(azimuths + 2 * M_PI, azimuths);


    float azimuth_min = azimuths.minCoeff();
    float azimuth_max = azimuths.maxCoeff();


    if ((azimuth_max - azimuth_min) > M_PI)
    {
        azimuth_min = (azimuths > (float)M_PI).select(azimuths, 10).minCoeff();
        azimuth_max = (azimuths < (float)M_PI).select(azimuths, 0).maxCoeff();
    }


   struct angle_extent angle_extent = { azimuth_min, azimuth_max, zeniths.minCoeff(), zeniths.maxCoeff() };
   /*
    printf("                      Extent: Azimuth = [%.2f, %.2f], Zenith = [%.2f, %.2f] (rad)\n", \
                                  angle_extent_face.azimuth_min, \
                                  angle_extent_face.azimuth_max, \
                                  angle_extent_face.zenith_min, \
                                  angle_extent_face.zenith_max);
   */

    return angle_extent;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool IntersectTriangle(const Vector3f& orig, const Vector3f& dir,
    Vector3f& v0, Vector3f& v1, Vector3f& v2,
    float* t, float* u, float* v)
{
    // E1
    Vector3f E1 = v1 - v0;

    // E2
    Vector3f E2 = v2 - v0;

    // P
    Vector3f P = dir.cross(E2);

    // determinant
    float det = E1.dot(P);

    // keep det > 0, modify T accordingly
    Vector3f T;
    if (det > 0)
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.0001f)
        return false;

    // Calculate u and make sure u <= 1
    *u = T.dot(P);
    if (*u < 0.0f || *u > det)
        return false;

    // Q
    Vector3f Q = T.cross(E1);

    // Calculate v and make sure u + v <= 1
    *v = dir.dot(Q);
    if (*v < 0.0f || *u + *v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    *t = E2.dot(Q);
    if (*t < 0.0f)
        return false;

    float fInvDet = 1.0f / det;
    *t *= fInvDet;
    *u *= fInvDet;
    *v *= fInvDet;

    return true;
}

MatrixXi get_exit_face(Crowncube _block[], Eigen::Vector3f _scanner_position, int _block_num)
{
    MatrixXi outer_face(_block_num * 3, 2); //外向面
    //获取所有外向面
    int num_outer = 0;
    for (int i = 0; i < _block_num; i++)
    {
        if (!_block[i].is_empty())
        {
            for (int j = 0; j < 6; j++)
            {
                //遍历每一个外向面
                if (_block[i].face_sign()[j])
                {
                    outer_face(num_outer, 0) = i;
                    outer_face(num_outer, 1) = j;
                    num_outer++;
                }
            }
        }
    }

    //outer_face.conservativeResize(num_outer, 2);
    MatrixXi exit_face(num_outer, 2); //穿出面

    //获取所有穿出面
    //使用区域坐标系  bn block_ID fn face_ID
    Vector3f dir;
    //float d = 0.0;
    int num_exit = 0;
    for (int m = 0; m < num_outer; m++)
    {
        int bn = outer_face(m, 0);
        int fn = outer_face(m, 1);
        dir = _block[bn].face_center()[fn] - _scanner_position;
        //核心思想，计算scan中心和当前面中心射线与外向面交点距离
        //有奇数个距离小于d（等价有奇数个t小于1），则当前面为exit_face
        int dis_low = 0;
        MatrixXf _face_vertices(4, 3);
        Vector3f v0, v1, v2, v3;
        float t, u, v = 0.0;
        //float d = dir.norm();
        for (int n = 0; n < num_outer; n++)
        {
            if (n == m)
            {
                continue;
            }
            int bn_ = outer_face(n, 0);
            int fn_ = outer_face(n, 1);
            //获得面四个顶点，并拆分成两个相切三角形
            _face_vertices = _block[bn_].face_point(fn_);
            v0 = _face_vertices.row(0).transpose();
            v1 = _face_vertices.row(1).transpose();
            v2 = _face_vertices.row(2).transpose();
            v3 = _face_vertices.row(3).transpose();
            //printf("_face_vertices : %f %f %f\n", v0[0], v0[1], v0[2]);
            if (IntersectTriangle(_scanner_position, dir, v0, v1, v2, &t, &u, &v))
            {
                //printf("t=%f\n", t);
                if (t > 0.0 && t < 1.0)
                {
                    dis_low++;
                    continue;
                }         
            }
            else if (IntersectTriangle(_scanner_position, dir, v2, v3, v0, &t, &u, &v) && (t > 0.0 && t < 1.0))
            {
                dis_low++;
            }
        }
        if (dis_low % 2 == 1)
        {
            //判定为exit面，输出
            exit_face(num_exit, 0) = bn;
            exit_face(num_exit, 1) = fn;
            num_exit++;
        } 
    }

    return exit_face.topRows(num_exit);
}

float get_paths(Vector2i _exit_face, Crowncube _block[], int _block_num, Eigen::Vector3f _scanner_position, Eigen::VectorXf& _path_ptx)
{
    
    int bn = _exit_face[0];
    int fn = _exit_face[1];
    float _Path_len = 0.0;
    Vector3f ori = _scanner_position;
    Vector3f dir = _block[bn].face_center()[fn] - ori; 
   
    for (int i = 0; i < _block_num; i++)
    {
        float _path = 0.0;
        //非空
        if (_block[i].is_empty())
        {
            _path_ptx[i] = -1.0;
        }
        else
        {
            //非空块，判断是否相交
            if (intersectWithAABB(_block[i], ori, dir, _path))
            {
               //相交,记录path_length
                _path_ptx[i] = _path;
                _Path_len += _path;
            }
            else
                _path_ptx[i] = 0.0;
        }
    }

    return _Path_len;
}

bool intersectWithAABB(Crowncube& _block, Eigen::Vector3f _ori, Eigen::Vector3f _dir, float& _path)
{
    Vector3f _xyz_min = _block.xyz_min();
    Vector3f _xyz_max = _block.xyz_max();
    float tmin = 0.0f;
    float tmax = FLT_MAX;

    //The plane perpendicular to x-axie
    //若射线方向矢量的x轴分量为0且原点不在盒体内
    if (abs(_dir[0]) < 0.000001f) //If the ray parallel to the plane
    {
        //If the ray is not within AABB box, then not intersecting
        if (_ori[0] < _xyz_min[0] || _ori[0] > _xyz_max[0])
            return false;
    }
    else
    {
        //Compute the distance of ray to the near plane and far plane
        float ood = 1.0f / _dir[0];
        float t1 = (_xyz_min[0] - _ori[0]) * ood;
        float t2 = (_xyz_max[0] - _ori[0]) * ood;

        //Make t1 be intersecting with the near plane, t2 with the far plane
        if (t1 > t2)
        {
            float temp = t1;
            t1 = t2;
            t2 = temp;
        }

        //Compute the intersection of slab intersection intervals
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;

        //Exit with no collision as soon as slab intersection becomes empty
        if (tmin > tmax) return false;
    }// end for perpendicular to x-axie

    //The plane perpendicular to y-axie
    if (abs(_dir[1]) < 0.000001f) //If the ray parallel to the plane
    {
        //If the ray is not within AABB box, then not intersecting
        if (_ori[1] < _xyz_min[1] || _ori[1] > _xyz_max[1])
            return false;
    }
    else
    {
        //Compute the distance of ray to the near plane and far plane
        float ood = 1.0f / _dir[1];
        float t1 = (_xyz_min[1] - _ori[1]) * ood;
        float t2 = (_xyz_max[1] - _ori[1]) * ood;

        //Make t1 be intersecting with the near plane, t2 with the far plane
        if (t1 > t2)
        {
            float temp = t1;
            t1 = t2;
            t2 = temp;
        }

        //Compute the intersection of slab intersection intervals
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;

        //Exit with no collision as soon as slab intersection becomes empty
        if (tmin > tmax) return false;
    }// end for perpendicular to y-axie

    //The plane perpendicular to z-axie
    if (abs(_dir[2]) < 0.000001f) //If the ray parallel to the plane
    {
        //If the ray is not within AABB box, then not intersecting
        if (_ori[2] < _xyz_min[2] || _ori[2] > _xyz_max[2])
            return false;
    }
    else
    {
        //Compute the distance of ray to the near plane and far plane
        float ood = 1.0f / _dir[2];
        float t1 = (_xyz_min[2] - _ori[2]) * ood;
        float t2 = (_xyz_max[2] - _ori[2]) * ood;

        //Make t1 be intersecting with the near plane, t2 with the far plane
        if (t1 > t2)
        {
            float temp = t1;
            t1 = t2;
            t2 = temp;
        }

        //Compute the intersection of slab intersection intervals
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;

        //Exit with no collision as soon as slab intersection becomes empty
        if (tmin > tmax) return false;
        //xyz230420Path约束
        if (tmin > 1) return false;
    }// end for perpendicular to z-axie

    _path = (tmax - tmin) * _dir.norm();
    return true;
}// end for intersectWithAABB</span>

MatrixXf get_face_dir(angle_extent _face_extent, MatrixXf& _directions, ArrayXf& _min_azimuth_col, ArrayXf& _max_azimuth_col, ArrayXf& _min_zenith_row, ArrayXf& _max_zenith_row, int _image_rows, int _image_cols, face_inf& _face_inf)
{
    int index_start_col, index_end_col, index_start_row, index_end_row;  // index of angle extent in raw data. [ index_start_* , index_end_* ) 
    int index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2; //temp index
    int index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2; //temp index
    
    (_min_azimuth_col - _face_extent.azimuth_min).abs().minCoeff(&index_min_azimuth_1);
    (_min_azimuth_col - _face_extent.azimuth_max).abs().minCoeff(&index_max_azimuth_1);
    (_max_azimuth_col - _face_extent.azimuth_min).abs().minCoeff(&index_min_azimuth_2);
    (_max_azimuth_col - _face_extent.azimuth_max).abs().minCoeff(&index_max_azimuth_2);

    (_min_zenith_row - _face_extent.zenith_min).abs().minCoeff(&index_min_zenith_1);
    (_min_zenith_row - _face_extent.zenith_max).abs().minCoeff(&index_max_zenith_1);
    (_max_zenith_row - _face_extent.zenith_min).abs().minCoeff(&index_min_zenith_2);
    (_max_zenith_row - _face_extent.zenith_max).abs().minCoeff(&index_max_zenith_2);

    // Row range
    index_start_row = Array4i(index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2).minCoeff() - 1;
    index_end_row = Array4i(index_min_zenith_1, index_min_zenith_2, index_max_zenith_1, index_max_zenith_2).maxCoeff() + 1;
    if (index_start_row < 0) index_start_row = 0;
    if (index_end_row > _image_rows - 1) index_end_row = _image_rows - 1;
    int n_rows = index_end_row - index_start_row + 1;
    // Colume range
    index_start_col = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).minCoeff() - 1;
    index_end_col = Array4i(index_min_azimuth_1, index_min_azimuth_2, index_max_azimuth_1, index_max_azimuth_2).maxCoeff() + 1;

    if (index_start_col < 0) index_start_col = 0;
    if (index_end_col > _image_cols - 1) index_end_col = _image_cols - 1;
    int n_cols = index_end_col - index_start_col +1;
    
    MatrixXf _dir_face(n_rows * n_cols, 3);
    //_face_dir_row = n_rows;
    //_face_dir_col = n_cols;
   
    int cnt = 0;
    for (int col = index_start_col; col <= index_end_col; col++)
    {
        for (int row = index_start_row; row <= index_end_row; row++)
        {
            _dir_face(cnt, 0) = _directions(col * _image_rows + row, 0);
            _dir_face(cnt, 1) = _directions(col * _image_rows + row, 1);
            _dir_face(cnt, 2) = _directions(col * _image_rows + row, 2);
            cnt++;
        }
    }

    _face_inf.face_rows = n_rows;
    _face_inf.start_row = index_start_row;
    _face_inf.end_row = index_end_row;
    _face_inf.face_cols = n_cols;
    _face_inf.start_col = index_start_col;
    _face_inf.end_col = index_end_col;

    return _dir_face;
}

Eigen::VectorXf get_Gap(Crowncube _block[], MatrixXf& _exit_vertices, MatrixXf& _exit_dir, int _block_num, Eigen::Vector3f _scanner_position, int _bn, int _fn)
{
    //后期专门获取outer_face作为输入变量
    MatrixXi outer_face(_block_num * 3, 2); //外向面
   //获取所有外向面
    int num_outer = 0;
    for (int i = 0; i < _block_num; i++)
    {
        if (!_block[i].is_empty())
        {
            for (int j = 0; j < 6; j++)
            {
                //遍历每一个外向面
                if (_block[i].face_sign()[j])
                {
                    outer_face(num_outer, 0) = i;
                    outer_face(num_outer, 1) = j;
                    num_outer++;
                }
            }
        }
    }
    
   
    MatrixXf _enter_vertices(4, 3);
    for (int m = 0; m < 4; m++)
    {
        Vector3f dir = _exit_vertices.row(m).transpose();
        MatrixXf _outer_vertices(4, 3);
        Vector3f v0, v1, v2, v3;
        float t, u, v = 0.0;
        float t_min = 1.0;
        for (int n = 0; n < num_outer; n++)
        {
            int bn_ = outer_face(n, 0);
            int fn_ = outer_face(n, 1);
            if (bn_ == _bn && fn_ == _fn)
            {
                continue;
            }
            //获得当前outer_face四个顶点，并拆分成两个相切三角形
            _outer_vertices = _block[bn_].face_point(fn_).rowwise()- _scanner_position.transpose();
            v0 = _outer_vertices.row(0).transpose();
            v1 = _outer_vertices.row(1).transpose();
            v2 = _outer_vertices.row(2).transpose();
            v3 = _outer_vertices.row(3).transpose();
            
            if (IntersectTriangle(Vector3f::Zero(3), dir, v0, v1, v2, &t, &u, &v))
            {
                //printf("t=%f\n", t);
                //if (t > 0.0 && t < 1.0)
                //{
                    t_min = min(t_min, t);
                //}
            }
            else if (IntersectTriangle(Vector3f::Zero(3), dir, v2, v3, v0, &t, &u, &v))
            {
                //if (t > 0.0 && t < 1.0)
                //{
                    t_min = min(t_min, t);
                //}
            }
        }
       
        _enter_vertices.row(m).transpose() = t_min * dir;
    }

    //获取了_enter_face,_exit_face,开始计算间隙率
    //遍历_exit_dir,计算进入_exit_face的点，enter_num
    int extent_num = 0; //exit_face约束范围内点数
    int enter_num = 0;
    int gap_num = 0;   //间隙点数

    Vector3f v0_enter, v1_enter, v2_enter, v3_enter;
    Vector3f v0_exit, v1_exit, v2_exit, v3_exit;

    v0_exit = _exit_vertices.row(0).transpose();
    v1_exit = _exit_vertices.row(1).transpose();
    v2_exit = _exit_vertices.row(2).transpose();
    v3_exit = _exit_vertices.row(3).transpose();

    v0_enter = _enter_vertices.row(0).transpose();
    v1_enter = _enter_vertices.row(1).transpose();
    v2_enter = _enter_vertices.row(2).transpose();
    v3_enter = _enter_vertices.row(3).transpose();

    for (int p = 0; p < _exit_dir.rows(); p++)
    {
        Vector3f dir = _exit_dir.row(p).transpose();
        float t_enter, u_enter, v_enter = 0.0;
        float t_exit, u_exit, v_exit = 0.0;
        float tt_enter, tt_exit = -1.0;
        
        //_exit_face有交点就是进入_exit_face的点
        if (IntersectTriangle(Vector3f::Zero(3), dir, v0_exit, v1_exit, v2_exit, &t_exit, &u_exit, &v_exit))
        {
            tt_exit = t_exit;
        }
        else if (IntersectTriangle(Vector3f::Zero(3), dir, v2_exit, v3_exit, v0_exit, &t_exit, &u_exit, &v_exit))
        {
            tt_exit = t_exit;
        }
        else
        {
            tt_exit = -1.0;
        }
        
        if (IntersectTriangle(Vector3f::Zero(3), dir, v0_enter, v1_enter, v2_enter, &t_enter, &u_enter, &v_enter))
        {
            tt_enter = t_enter;
        }
        else if (IntersectTriangle(Vector3f::Zero(3), dir, v2_enter, v3_enter, v0_enter, &t_enter, &u_enter, &v_enter))
        {
            tt_enter = t_enter;
        }
        else
        {
            tt_enter = -1.0;
        }

        //开始判断
        if (tt_exit < tt_enter) { continue; }                                                                         
        if (tt_enter < 0 || tt_exit < 0) { continue; }                                                                //未相交，未进入包络
        if (tt_enter > 1 && tt_exit > 1) extent_num++;                                                                //相交，在exit_face约束范围内但未进入包络，在包络前返回（遮挡）
        if (tt_enter > 0 && tt_enter < 1 && tt_exit > 1) { extent_num++; enter_num++; }                               //相交，进入包络，在包络内返回（树冠点）
        if (tt_enter > 0 && tt_enter < 1 && tt_exit > 0 && tt_exit < 1) { extent_num++;  enter_num++; gap_num++; }    //相交，进入包络，在包络后返回（间隙点）

    }
    
    VectorXf _Gap_Weight(2);
    _Gap_Weight(1) = enter_num;

    if (enter_num == 0)  _Gap_Weight(0) = 1.0;
    else                 _Gap_Weight(0) = (gap_num * 1.0) / (enter_num * 1.0);

   return _Gap_Weight;
    
}