1.Extract the file 'TACT-exe.7z' to the current folder;

2.Open the 'CT_LAI.bat' file with Notepad and update the data paths to the correct ones, particularly for the PTX, ENVELOPE, and g_Mes_   5.txt files;

3.Choose an appropriate -xy_scale, which determines how many voxels the x and y dimensions of the tree canopy bounding box are divided into, as this directly affects the voxel size;

4.Save and close the .bat file, then double-click to start the calculation;

5.For first-time use, we recommend using the parameter '-img_debug' to generate debug images, which can be used to check if the data reading code has correctly extracted the target canopy point cloud from the .ptx file. The data reading code in the current version may be affected by different TLS systems and scanning methods.

