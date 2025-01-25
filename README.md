<p align="center">

  <h1 align="center">Wheel-GINS: A GNSS/INS Integrated Navigation System with a Wheel-mounted IMU</h1>

  <p align="center">
    <a href="https://arxiv.org/pdf/2501.03079"><img src="https://img.shields.io/badge/Paper-pdf-<COLOR>.svg?style=flat-square" /></a>
    <a href="https://github.com/i2Nav-WHU/Wheel-GINS"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a> <a href="https://github.com/i2Nav-WHU/Wheel-GINS"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a>
    <a href="https://github.com/i2Nav-WHU/Wheel-GINS/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg?style=flat-square" /></a> 
    
  </p>

</p>

**Wheel-GINS is an integrated navigation system that fuses GNSS with a wheel-mounted IMU (Wheel-IMU). Building on insights from our previous study, [Wheel-INS](https://github.com/i2Nav-WHU/Wheel-INS), which highlighted the significant advantages of the Wheel-IMU, we propose Wheel-GINS to further enhance localization accuracy and robustness. It performs a similar information fusion process as the conventional GNSS/Odometer/IMU integrated systems. Moreover, all Wheel-IMU installation parameters are estimated online in Wheel-GINS, enhancing the system's practicality and adaptability.**

![wheelgins_overview](https://github.com/user-attachments/assets/aa31d91c-9b43-4287-8f17-a63c124285d7)


## Run it
### 1. Linux
We recommend you to use g++ compiler with Ubuntu 20.04. The build-essential libraries should be installed first:
```shell
sudo apt-get install cmake
sudo apt-get install build-essential
```

After preparing the compilation environment, you can clone the repository and run it as follows:

```shell
# Clone the repository
git clone git@github.com:i2Nav-WHU/Wheel-GINS.git ~/

# Build Wheel-INS
cd ~/Wheel-GINS
mkdir build && cd build

cmake ..
make -j10

# Run demo dataset
cd ~/Wheel-INS
./bin/Wheel-GINS config/20231213.yaml
```
### 2. Windows
Here we show how to run the code with [Visual Studio Code (VSCode)](https://code.visualstudio.com/), but you can also use other IDEs, e.g., [Visual Studio](https://visualstudio.microsoft.com/).

- Install VSCode and the extensions: **C/C++**, **C/C++ Extension Pack**, **CMake**, and **CMake Tools**.
- Install [CMake](https://cmake.org/download/) and [Microsoft Visual C/C++ Build Tools](https://visualstudio.microsoft.com/downloads/).
- Open Wheel-INS with VSCode.
- Set compiler: open the Command Palette (Ctrl+Shift+P) and type "CMake: Select a Kit", select the correct build tool according to your system.
- Configure CMake: type and click "CMake: Configure" in the Command Palette.
- Compile Project: type and click "CMake: Build" in the Command Palette.

Once a execuble file **Wheel-INS.exe** is generated, the compilation is done. Then, you can run it via the terminal in VSCode as following:

```shell
.\bin\Release\Wheel-GINS.exe config/20231213.yaml
```


## Related Papers
If you find our studies helpful to your academic research, please consider citing the related papers.

- Y. Wu, J. Kuang, X. Niu, C. Stachniss, L. Klingbeil and H. Kuhlmann, "Wheel-GINS: A GNSS/INS Integrated Navigation System with a Wheel-mounted IMU," IEEE Transactions on Intelligent Transportation Systems, 2025. 
```bibtex
@ARTICLE{wu2025tits,
  author={Wu, Yibin and Kuang, Jian and Niu, Xiaoji and Stachniss, Cyrill and Klingbeil, Lasse and Kuhlmann, Heiner},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={{Wheel-GINS}: A {GNSS/INS} Integrated Navigation System with a Wheel-mounted {IMU}}, 
  year={2025}
}
```

- X. Niu, Y. Wu and J. Kuang, "Wheel-INS: A Wheel-mounted MEMS IMU-based Dead Reckoning System," IEEE Transactions on Vehicular Technology, doi: 10.1109/TVT.2021.3108008, 2021. ([pdf](http://i2nav.cn/ueditor/jsp/upload/file/20210905/1630804325780076093.pdf)) ([code](https://github.com/i2Nav-WHU/Wheel-INS))
```bibtex
@ARTICLE{niu2021tvt,
  author={Niu, Xiaoji and Wu, Yibin and Kuang, Jian},
  journal={IEEE Transactions on Vehicular Technology}, 
  title={{Wheel-INS}: A Wheel-Mounted {MEMS IMU}-Based Dead Reckoning System}, 
  year={2021},
  volume={70},
  number={10},
  pages={9814-9825},
  doi={10.1109/TVT.2021.3108008}
}
```

## License
The code is released under GPLv3 license.

## Acknowledgement
Thanks to [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS). 

For any questions, please feel free to contact Mr. Yibin Wu (ybwu at whu.edu.cn) or Dr. Jian Kuang (kuang at whu.edu.cn).