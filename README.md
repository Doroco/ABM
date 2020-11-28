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

![DWA_SIM](https://user-images.githubusercontent.com/49723556/100526580-6ecfae00-320d-11eb-85dc-180bc0399903.gif)

- 3-DOF Inverse Kinematics

- Ros와 MCU간의 Serial 통신

### Project Scenarios


- 세부기술 정리: https://doroco.github.io/ 

### reference
- [1]Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance. IEEE Robotics & Automation Magazine, 4(1), 23–33. 
- [2]강형석, 「제어기 설계를 위한 DC 모터의 모델 파라미터 측정 및 실험적 보정」, 『한국정밀공학회지』, 제 31권 12호, 한국정밀공학회, pp1147-1154.
김상훈 저,⒊『(DC,AC,BLDC)모터제어』, 서울 : 복두출판사, 2010,pp54-115.
https://pinkwink.kr/732
- [3]Rached Dhaouadi, Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework,Adv Robot Autom,2013,pp3-6.
- [4]Modern Robotics, Kevin M.Lynch, Frank C.Park
