# Project PointCloud On Image

code for projecting a 3D point cloud to a 2D image using the intrinsic parameters of a camera as well as its extrinsic parameters.

## Project Structure

* [`config`](./config/) : Contains yaml file
* [`include`](./include/): Contains yaml parser and projection header
* [`output`](./output/) : To store output
* [`resources`](./resources/): Contains example image and pcd file
* [`src`](./src): Implementation and example code.

## Dependencies

Latest dependencies

* `OpenCV`
* `PCL`
* `yaml-cp`

## config

Change config parameter accordingly

* `extrinsicTranslation` or `Tr` : 3 x 4 extrinsic camera matrix

* `projectionMatrix` or `P0` : 3 x 4 projection matrix

## Build instruction

```bash
git clone https://github.com/Basavaraj-PN/project-pointcloud-on-image.git
cd project-pointcloud-on-image
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Running Example

Run pointcloud_to_image in `build` directory

```bash
./pointcloud_to_image
```

## Result on KITTY dataset

![alt text](https://github.com/Basavaraj-PN/project-pointcloud-on-image/blob/main/output/000000.png)
