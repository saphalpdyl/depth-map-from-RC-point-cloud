# What is this ?
This is a fun little project for calculating depth from a point cloud export from Reality Capture 1.5.

~~_Note: The depth part is still in the making_~~ Done!

## How to use your own data?
Currently, you can run the program to render the point cloud on `sample` data that I included of my little workspace. However, if you want to experiment with you own data, you require the following files/exports:
- Point cloud in `.ply` format
- Images folder in `.heif` format
- Camera calibration data folder in `.xmp` format
- ~~Camera parameters in `.csv` format~~ ( _No longer required since metadatas files are primarily used for camera intrinsics and extrinsics_ )

---
#### Point cloud projection + Depth map
![Projection + Depth Map Figure 1](https://github.com/user-attachments/assets/c43f16f2-368f-41e7-a6ba-621acf8661ba)
![Projection + Depth Map Figure 1](https://github.com/user-attachments/assets/5c264dec-bf65-4be5-b10f-150447a31601)


#### Sparse Point Cloud
![Projection with Sparse point cloud](https://github.com/user-attachments/assets/d5a465be-9c69-4095-9a49-c983ae3c81e3)

#### Dense Point Cloud
![Projection with Dense point cloud](https://github.com/user-attachments/assets/fea0214b-85eb-4d2c-b59f-a365376fa0ea)


## Sources
- https://skannai.medium.com/projecting-3d-points-into-a-2d-screen-58db65609f24 _Projecting 3D Points into a 2D Screen_
