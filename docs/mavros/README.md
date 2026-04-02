# mavros-GeographicLib

mavros 所需的 GeographicLib 数据集及 PX4 启动文件。

## 文件说明

| 文件/目录 | 说明 |
|---|---|
| `GeographicLib/geoids/egm96-5.*` | 大地水准面模型数据集 |
| `GeographicLib/gravity/egm96.*` | 重力模型数据集 |
| `GeographicLib/magnetic/emm2015.*` | 磁场模型数据集 |
| `GeographicLib/px4.launch` | PX4 的 mavros 启动文件 |

## GeographicLib 数据集放置位置

将 `GeographicLib/` 文件夹中的三个子目录（`geoids`、`gravity`、`magnetic`）复制到以下路径之一（需要 root 权限）：

```
/usr/share/GeographicLib/
```
或
```
/usr/local/share/GeographicLib/
```

最终目录结构应为：

```
/usr/share/GeographicLib/
├── geoids/
│   ├── egm96-5.pgm
│   ├── egm96-5.pgm.aux.xml
│   └── egm96-5.wld
├── gravity/
│   ├── egm96.egm
│   └── egm96.egm.cof
└── magnetic/
    ├── emm2015.wmm
    └── emm2015.wmm.cof
```

复制命令示例：

```bash
sudo cp -r GeographicLib/geoids   /usr/share/GeographicLib/
sudo cp -r GeographicLib/gravity  /usr/share/GeographicLib/
sudo cp -r GeographicLib/magnetic /usr/share/GeographicLib/

sudo bash install_geographiclib_local.sh

```

> 也可以运行仓库中的 `install_geographiclib_datasets.sh` 脚本自动下载安装（需要联网）：
> ```bash
> sudo bash install_geographiclib_datasets.sh
> ```

