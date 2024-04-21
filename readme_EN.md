EN | [中文](readme.md)  
Attention: Translated by ChatGPT 3.5. There may be inaccuracies in expression.
# roi_dataset_flow
## Purpose
Select regions of interest (ROI) from large-area aerial oblique data to create a small dataset containing the ROI, facilitating subsequent modeling of individual buildings through image visibility screening and image cropping. 

* **Visibility Screening**  
   Determine whether the world points of the ROI are visible in the image by comparing the angle between the camera orientation and the line connecting the camera center and the world point with the size of the camera field of view. If the proportion of ROI world points visible on an image exceeds a set threshold, the image is deemed necessary.

* **Mask-based Cropping**  
   After visibility screening, the images contain only the ROI. To focus on the ROI and reduce interference from other parts (e.g., if the ROI occupies only a small portion of the original image, downsampling for subsequent reconstruction may result in loss of effective information affecting reconstruction effectiveness), calculate the corresponding image points based on the ROI world points multiplied by the camera's extrinsic (w2c) and intrinsic (c2p) parameters. Generate a bounding box based on the extremities of the image point coordinates, and use this bounding box to crop the image (adopting certain strategies to ensure the cropped images have the same dimensions).

* **Generate camera parameter files for the new dataset in formats similar to nerf_studio and neuralangelo (.json)**

```
{
  "camera_mode": ,
  "camera_orientation": ,
  "aabb_scale": ,
  "aabb_range":,
  "sphere_center":,
  "sphere_radius":,
  "frames": [
  {
    "file_path": ,
    "intrinsic_matrix": ,
    "transform_matrix": ,
    "w": ,
    "h": 
  }, 
  ...
 }
```

## Preparation
* Download a large-scale aerial oblique dataset.
  Example: EuroSDR Benchmark for Multi-Platform Photogrammetry published by ISPRS.
  [Link](https://www2.isprs.org/commissions/comm2/icwg-2-1a/benchmark_main/)
* Create a new project in Context Capture Master, load the dataset, and perform aerial triangulation (skipping ground control points if the dataset quality is good).
* Export camera parameter files (AT.xml) and undistorted images after aerial triangulation.
* Create a 3D reconstruction project, select the area of interest, and generate the mesh (Model.obj) file and metadata file (metadata.xml) for that area.

## Data Organization
* Create a `dataset` folder and organize it as follows:

```
dataset
|_ dataset_01
   |_ images        // Path to store images visible to ROI initially empty
   |_ images_crop   // Path to store cropped images based on ROI initially empty
   |_ AT.xml
   |_ metadata.xml
   |_ Model.obj
```

## Code Structure
```
reader
|_ camera_reader.py // Reads camera parameters (AT.xml) information
|_ obj_reader.py    // Reads mesh file (Model.obj) information
scripts
|_ tools.py  // Some utility functions
|_ scene_visualizer.py  // Visualizes the scene and poses
run.py  
```

## Parameter Settings
In `run.py`, set:
* Dataset path `dataset_dir`
* Whether using a single camera `if_single_camera`
* Whether to crop images `if_mask_crop`
* Target size after image cropping `tar_size_w` `tar_size_h`
* Whether to normalize the scene to a specified range `if_standardization`
* Target radius of the scene `tar_radius`

Other input and output file paths need not be changed. Once set, run `run.py`.

## Considerations
* The output from CC consists of a `3x3` rotation matrix for w2c and the camera's `3x1` coordinates in the world coordinate system. When exporting, the camera coordinate axis orientation is selected as `opencv` (i.e., `RDF`).

## Test

* Dataset: Penta-Cam-Centre(8bit)
* ROI mesh: ![Image](assets/image-2.png)
* Example of an image after visibility screening (8176 * 6132): ![Image](assets/image.png)
* Cropped ROI from the image (1200 * 1000): ![Image](assets/image-1.png)
* Obtaining parameters for the new image by scaling the scene to a sphere with a radius of 1:

```
"file_path": "images/001_009_145000282.jpg",
"intrinsic_matrix": [
[
    13658.7021484375,
    0.0,
    -2580.524169921875
],
[
    0.0,
    13658.7021484375,
    323.05224609375
],
[
    0.0,
    0.0,
    1.0
]
],
"transform_matrix": [
[
    0.9999997615814209,
    0.00018038207781501114,
    -0.0006661160150542855,
    -3.903818368911743
],
[
    0.0006009129574522376,
    -0.7022261023521423,
    0.7119537591934204,
    -11.74245548248291
],
[
    -0.0003393403603695333,
    -0.7119539976119995,
    -0.7022260427474976,
    11.748698234558105
],
[
    0.0,
    0.0,
    0.0,
    1.0
]
],
"w": 1200,
"h": 1000
```

## References
* Dataset Information: ISPRS
* Visualization: NeRF++
* JSON File Format: 
  * NeRF_Studio 
  * Neuralangelo