# Autonomous driving DDMR agent  

로봇팔이 부착된 Mobile robot이 다양한 Task를 수행합니다.

- 프로젝트 5분 소개영상
https://youtu.be/9r_tR3T8_8s

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

![Global_PathPlanning](https://user-images.githubusercontent.com/54099930/109391138-f33ee380-7958-11eb-99e8-d518f9859364.gif)
![DWA_SIM](https://user-images.githubusercontent.com/49723556/100526580-6ecfae00-320d-11eb-85dc-180bc0399903.gif)

- 실제 주행

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/49723556/100527352-261bf300-3215-11eb-8e6c-c0235fb511b6.gif)

- 3-DOF Inverse Kinematics

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/49723556/100526648-472d1580-320e-11eb-9251-9d0de1e72fd4.gif)

- Ros와 MCU간의 Serial 통신

### Project Scenarios


- 세부기술 정리(준비중): https://doroco.github.io/ 

### reference
- [1]Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance. IEEE Robotics & Automation Magazine, 4(1), 23–33. 
- [2]강형석, 「제어기 설계를 위한 DC 모터의 모델 파라미터 측정 및 실험적 보정」, 『한국정밀공학회지』, 제 31권 12호, 한국정밀공학회, pp1147-1154.
김상훈 저,⒊『(DC,AC,BLDC)모터제어』, 서울 : 복두출판사, 2010,pp54-115.
https://pinkwink.kr/732
- [3]Rached Dhaouadi, Dynamic Modelling of Differential-Drive Mobile Robots using Lagrange and Newton-Euler Methodologies: A Unified Framework,Adv Robot Autom,2013,pp3-6.
- [4]Modern Robotics, Kevin M.Lynch, Frank C.Park
