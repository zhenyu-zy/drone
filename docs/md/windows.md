# Windows 端配置指南

### 一、NVIDIA 驱动安装与更新
1. **查看显卡版本**  
   首先查看电脑的显卡版本。  
   ![电脑的显卡版本](../alt/1.png)  
   如果已有显卡驱动，可以直接在桌面右键，找到英伟达驱动控制面板打开。  
   ![英伟达驱动控制面板](../alt/2.png)  
   显卡驱动的下载地址：[NVIDIA GeForce 驱动程序](https://www.nvidia.com/Download/index.aspx)  
   ![NVIDIA GeForce 驱动程序下载地址](../alt/3.png)

2. **检查驱动版本**  
   安装（更新）好显卡驱动后，按下 `Win + R` 组合键，打开命令窗口，输入以下命令：
   ```bash
   nvidia-smi
   ```
   例如，可以看到驱动版本为 `555.99`，最高支持的 CUDA 版本为 `12.5`。  
   ![nvidia-smi](../alt/4.png)

### 二、安装 CUDA 和 cuDNN
1. **下载 CUDA Toolkit**  
   下载地址：[CUDA Toolkit 12.8 Update 1](https://developer.nvidia.com/cuda-downloads)  
   ![CUDA Toolkit 下载](../alt/5.png)

2. **下载 cuDNN**  
   下载地址：[cuDNN 9.2.1](https://developer.nvidia.com/cudnn)  
   ![cuDNN 下载](../alt/6.png)  
   将 zip 文件解压。

### 三、Anaconda 的安装
1. **下载 Anaconda**  
   官网下载地址：[Anaconda Distribution](https://www.anaconda.com/products/individual)

2. **安装步骤**  
   - 下载安装程序并运行。
   - 按照提示完成安装（安装在默认路径）。

### 四、PyTorch 环境安装
1. **创建虚拟环境**  
   打开 Anaconda 终端，创建虚拟环境：
   ```bash
   conda create -n yolov5 python=3.8
   conda create -n yolov8 python=3.8
   ```
   此处虚拟环境名为 `yolov5(yolov8)`，Python 版本为 `3.8`。

2. **激活虚拟环境**  
   ```bash
   conda activate yolov5
   conda activate yolov8
   ```

3. **配置清华源（可选）**  
   如果需要加速下载，可以将 Anaconda 源切换为清华源：
   ```bash
   conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
   conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
   conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
   conda config --set show_channel_urls yes
   ```
   查看源：
   ```bash
   conda config --show
   ```
   恢复默认源：
   ```bash
   conda config --remove-key channels
   ```

4. **安装 PyTorch**  
   根据显卡支持的 CUDA 版本（如 CUDA 12.1），安装 PyTorch：
   ```bash
   conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
   ```
   如果可以尝试使用国内源：
   ```bash
   conda install pytorch torchvision torchaudio pytorch-cuda=12.1
   ```

5. **测试 PyTorch**  
   在 Python 中运行以下代码测试 PyTorch、CUDA 和 cuDNN 是否正常工作：
   ```python
   import torch
   print(torch.cuda.is_available())
   print(torch.backends.cudnn.is_available())
   print(torch.cuda_version)
   print(torch.backends.cudnn.version())
   ```

### 五、PyCharm 安装
1. **下载 PyCharm**  
   官网下载地址：[JetBrains PyCharm](https://www.jetbrains.com/pycharm/download/)

2. **安装步骤**  
   - 下载安装 PyCharm Community Edition 并运行（安装在默认路径）。
   - 按照提示完成安装。

### 六、LabelImg 安装及使用
1. **安装 LabelImg**  
   使用以下命令安装：
   ```bash
   pip install labelimg -i https://pypi.tuna.tsinghua.edu.cn/simple
   ```
   安装的路径为"C:\Users\ASUS\anaconda3\Scripts\labelImg.exe"

   - **界面介绍**  
     ![LabelImg 界面介绍](alt/labellmg1.png)

   - **设置（标签格式为 YOLO）**  
     ![LabelImg 设置](../alt/labellmg2.png)

   - **打开需要标注的图片文件夹，设置标注文件保存的目录 (Change Save Dir)**  
     ![LabelImg 文件夹](../alt/labellmg3.png)

   - **开始标注，画框，标记目标的 label**  
     ![LabelImg 文件夹](../alt/labellmg4.png)

   - **LabelImg 的快捷键**  
     ![LabelImg 快捷键](../alt/labellmg5.png)

### 七、项目克隆和环境依赖安装
1. **YOLOv5 项目**  
   - 克隆仓库：
     ```bash
     git clone https://github.com/ultralytics/yolov5.git
     ```
   - 安装依赖：
     ```bash
     cd yolov5
     pip install numpy==1.23.5 pillow==9.5
     pip install -r requirements.txt
     ```

2. **YOLOv8 项目**  
   - 克隆仓库：
     ```bash
     git clone -b v8.2.103 https://github.com/ultralytics/ultralytics.git
     ```
   - 安装依赖：
     ```bash
     cd yolov8
     pip install numpy==1.23.5 pillow==9.5
     pip install -r requirements.txt
     ```