# Sophus 李群库

### 一、克隆仓库

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
```

### 二、修改源码

- 修改 `Sophus/sophus/so2.cpp` 第 22 行，添加构造函数初始化：
  ```cpp
  SO2::SO2()
  {
    unit_complex_.real(1.);
    unit_complex_.imag(0.);
  }
  ```

### 三、编译安装

```bash
mkdir build && cd build
cmake ..
make -j8
sudo make install
```
