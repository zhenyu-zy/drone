# Deepstream-YOLO

### 1. PyTorch
- 安装依赖：
  ```bash
  sudo apt-get install python3-pip libopenblas-base libopenmpi-dev
  pip3 install Cython
  pip3 install numpy==1.19.3
  pip3 install protobuf==3.3.0
  pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
  ```

### 2. torchvision (v0.9.0)
- 安装依赖：
  ```bash
  sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
  python3 -m pip install --upgrade pillow
  ```
- 克隆并安装：
  ```bash
  git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
  cd torchvision
  export BUILD_VERSION=0.9.0
  python3 setup.py install --user
  ```

### 3. 清华源
- 使用清华源安装 Python 包：
  ```bash
  pip install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```

### 4. YOLOv5 (v6.2)
- 安装依赖：
  ```bash
  pip install testresources
  pip install launchpadlib
  ```
- 克隆并安装：
  ```bash
  git clone -b v6.2 https://github.com/ultralytics/yolov5.git
  pip install -r requirements.txt
  ```

### 5. TensorRTX (YOLOv5 v6.2)
- 克隆并编译：
  ```bash
  git clone -b yolov5-v6.2 https://github.com/wang-xinyu/tensorrtx.git
  cd tensorrtx
  mkdir build
  cd build
  cmake ..
  make
  # 修改 yololar.h 中的 class_num
  ```

### 6. YOLOv8 (v8.2.103)
- 克隆并安装：
  ```bash
  git clone -b v8.2.103 https://github.com/ultralytics/ultralytics.git
  pip install -r requirements.txt
  ```

### 7. Deepstream 6.0.1
- **第一种**：用 TensorRT 转换 wts 和 cfg，使用这个版本的 DeepStream-YOLO  
  使用脚本生成权重文件：
  ```bash
  # gen_wts_yolov5.py
  ```
  [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo/tree/e652ef4e394fbcee0b8b8652c4630802bec4eab3)

- **第二种**：使用 ONNX  
  克隆仓库：
  ```bash
  git clone https://github.com/marcoslucianops/DeepStream-Yolo.git
  ```
  配置文件：`config_infer_primary_yoloV5.txt`  
  ONNX 模型文件：
  ```ini
  onnx-file=yolov5s.onnx
  ```
  生成的引擎文件：
  ```ini
  model-engine-file=model_b1_gpu0_fp32.engine
  ```
  标签文件：
  ```ini
  labelfile-path=labels.txt
  ```
  置信度：
  ```ini
  pre-cluster-threshold=0.25
  ```
  配置文件：`deepstream_app_config.txt`
  ```ini
  [source0]
  enable=1
  type=3
  uri=file:///opt/nvidia/deepstream/deepstream/samples/streams/sample_1080p_h264.mp4
  num-sources=1
  gpu-id=0
  cudadec-memtype=0

  [primary-gie]
  config-file=config_infer_primary_yoloV5.txt
  ```

### 8. Deepstream-python
- 相关配置和安装步骤
