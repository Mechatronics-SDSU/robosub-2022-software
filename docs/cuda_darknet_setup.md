# Darknet setup instructions

### CUDA
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions


### CUDNN
From: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux-tar
Run the following commands (do not use above link):

`sudo cp usr/include/cudnn*.h /usr/local/cuda/include`
`sudo cp -P /usr/lib/aarch64-linux-gnu/libcudnn* /usr/local/cuda/lib64`
`sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*`

### Variables:

`export PATH=$PATH:/usr/local/cuda/bin`

`export LD_LIBRARY_PATH=/usr/local/cuda/lib64`

`export CUDACXX=/usr/local/cuda/bin`

`export CUDA_PATH=/usr/local/cuda`


### Clone Darknet Git
https://github.com/AlexeyAB/darknet


### Make Instructions 
https://pjreddie.com/darknet/install/

### Darknet Setup
https://pjreddie.com/darknet/yolo/

### Tensorflow
https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html
(Use sudo -H python3 -m pip)
