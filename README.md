# Autonomous driving DDMR agent  

로봇팔이 부착된 Mobile robot이 다양한 Task를 수행합니다.


## 목차

- System Architecture
- Hardware Architecture
- 구현내용
- Project Secenarios
- Reference


### System Architecure
![시스템아키텍쳐](https://user-images.githubusercontent.com/49723556/100517133-f7c5f580-31cb-11eb-95bd-975f90b0b69b.png)

### Hardware Achitecture
![하드웨어 아키텍쳐](https://user-images.githubusercontent.com/49723556/100517722-2e9e0a80-31d0-11eb-907d-737a6c4b231f.png)

### 구현내용

- 모터제어

- DWA(Dynamic Window Approach)

- 3-DOF Inverse Kinematics

- Ros와 MCU간의 Serial 통신

### Project Scenarios


- 세부기술 정리: https://doroco.github.io/ 

### reference
- [1] Borenstein, J., & Koren, Y. (1991). The vector field histogram-fast obstacle avoidance for mobile robots. IEEE Transactions on Robotics and Automation, 7(3), 278–288. doi:10.1109/70.88137 
- [2] Ulrich, I., & Borenstein, J. (n.d.). VFH+: reliable obstacle avoidance for fast mobile robots. Proceedings. 1998 IEEE International Conference on Robotics and Automation (Cat. No.98CH36146). doi:10.1109/robot.1998.677362 
- [3] Ulrich, I., & Borenstein, J. (n.d.). VFH/sup */: local obstacle avoidance with look-ahead verification. Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference on Robotics and Automation. Symposia Proceedings (Cat. No.00CH37065). doi:10.1109/robot.2000.846405 
- [4] LaValle, Steven M. (October 1998). "Rapidly-exploring random trees: A new tool for path planning" (PDF). Technical Report. Computer Science Department, Iowa State University (TR 98–11).
- [5]Adiyatov, O., & Varol, H. A. (2017). A novel RRT*-based algorithm for motion planning in Dynamic environments. 2017 IEEE International Conference on Mechatronics and Automation (ICMA). doi:10.1109/icma.2017.8016024 
- [6]강형석, 「제어기 설계를 위한 DC 모터의 모델 파라미터 측정 및 실험적 보정」, 『한국정밀공학회지』, 제 31권 12호, 한국정밀공학회, pp1147-1154.
김상훈 저,⒊『(DC,AC,BLDC)모터제어』, 서울 : 복두출판사, 2010,pp54-115.
https://pinkwink.kr/732
- [7]Rached Dhaouadi, Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework,Adv Robot Autom,2013,pp3-6.
