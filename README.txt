The Tree Canopies Computed Tomography (TACT) model was proposed by Yuzhen Xing, Ronghai Hu, and others. Inspired by the Computed Tomography principle, the TACT model was developed to directly inverts the complete 3D distribution of plant area volume density within the tree canopy using multi-angle terrestrial laser scanning data.Data is publicly available at https://zenodo.org/records/14756145. The related paper is currently under submission. For any questions or support, please feel free to contact xingyuzhen20@mails.ucas.ac.cn or huronghai@ucas.ac.cn.

1. This project is developed using Microsoft Visual Studio 2019 with C++ on the Windows 10 operating system. PCL_1.12.1 is used for visualization, so please ensure that PCL_1.12.1 is correctly configured before use. For detailed instructions, it is recommended to refer to https://blog.csdn.net/syz201558503103/article/details/103892364 and https://blog.csdn.net/as_your_heart/article/details/125692805.

2. Enter parameters via the command line:

(a) Project - Properties - Debugging - Working Directory: E:\DATA\AT01 (the location of the data. For the detailed folder structure, please refer to the sample data.)

(b) Project - Properties - Debugging - Command parameters: -ptx 'PTX*.ptx' -envelope 'ENVELOPE*.pcd' -g g_Mes_5.txt 18 -xy_scale 30 -img_debug

-ptx 'PTX*.ptx' :              use all .ptx files in the PTX folder for the calculation;

-envelope 'ENVELOPE*.pcd' :    perform the calculation on each tree canopy envelope in the ENVELOPE folder, ensuring they correspond to the PTX data;

-g g_Mes_5.txt 18 :            'g_Mes_5.txt' is used to calculate the G value, detailed description can be found in the paper;

-xy_scale 30 :                 -xy_scale determines how many voxels the x and y dimensions of the tree canopy bounding box are divided into, as this directly affects the voxel size;

-img_debug :                   for first-time use, we recommend using the parameter '-img_debug' to generate debug images, which can be used to check if the data reading code has correctly extracted the target canopy point cloud from the .ptx file. The data reading code 
                               in the current version may be affected by different TLS systems and scanning methods.

3. We have provided executable files and running scripts in the Quick-use section, allowing for a faster and more direct understanding of TACT when combined with our open-source data.

4. After running the program, a .txt file named after the xy_scale input parameter will be generated, which contains detailed information about voxel size, individual tree plant area, and the 3D plant area volume density distribution.
