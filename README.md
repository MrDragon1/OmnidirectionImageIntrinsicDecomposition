# Intrinsic Omnidirectional Image Decomposition with Illumination Pre-Extraction

This is the official implementation of our TVCG paper **"Intrinsic Omnidirectional Image Decomposition with Illumination Pre-Extraction"**.

## Usage

To generate the results of our paper, please the following steps:

+ Put the input image into *img* folder, as well as the depth and segment image.
+ Modify *main.m* and *Our.m* to make sure the input images are read correctly.
+ Run *main.m*, and the results will be saved in *img/results*  folder.
+ You can modify the *param* to run ablation.

Note that you can use other segmentation methods and modify the *ceiling_color* and *floor_color* in *Our.m*.
