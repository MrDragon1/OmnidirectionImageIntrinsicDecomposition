# Intrinsic Omnidirectional Image Decomposition with Illumination Pre-Extraction

This is the official implementation of our TVCG paper **"Intrinsic Omnidirectional Image Decomposition with Illumination Pre-Extraction"**.

## Usage

To generate the results of our paper, just run *main.m*. The results will be saved in *img/results* folder.

If you want to run with your own image, please the following steps:

+ Put the input image into *img* folder, as well as the depth and segment image.
+ Modify *main.m* and *Our.m* to make sure the input images are read correctly (.npy or .png are supported here).
+ Run *main.m*, and the results (including reflectance, shading, light source and pre-extraction image) will be saved in *img/results* folder.
+ You can modify the *param* to run ablation.

Note that you can use other segmentation methods and modify the *ceiling_color* and *floor_color* in *Our.m* to identify the floor and ceiling correctly.
