#  4족 보행 로봇 (Quadruped-Robot)

**4족 보행 로봇 프로젝트**는 STM32와 PCA9685 16채널 PWM 드라이버, MG995 서보모터를 이용해  
4족 보행 로봇의 **전진, 후진, 좌/우(측면) 이동** 기능을 구현한 프로젝트입니다.

---

##  프로젝트 개요

- **수행 기간**: 2024.07.23 ~ 2024.12.30  
- **사용 기술**:  
  - STM32  
  - C  
  - I2C 통신  
  - PWM 제어  
  - **파이썬 기반 시뮬레이션**

- **주요 기능**:  
  - STM32와 I2C로 서보모터 제어  
  - 4족 로봇 전진/후진/좌/우(측면) 이동 보행

---

##  기술 스택

| 기술  | 설명 |
|---|---|
| ![STM32](https://img.shields.io/badge/STM32-0076D6?style=flat&logo=STMicroelectronics&logoColor=white) | 4족 보행 로봇 메인 MCU |
| ![C](https://img.shields.io/badge/C-A8B9CC?style=flat-square&logo=c&logoColor=white) | 보행 알고리즘 및 제어 로직 구현 |
| ![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white) | **로봇 보행 시뮬레이션 및 변수 효과 검증** |
| ![I2C](https://img.shields.io/badge/I2C-000000?style=flat&logo=internet-explorer&logoColor=white) | PCA9685 16채널 PWM 드라이버 제어 |


---

##  기능 설명

- **전진, 후진, 좌/우 이동**  
  - 각 다리의 보행 알고리즘을 통해 전진/후진 뿐만 아니라,  
    회전 없이 **왼쪽/오른쪽(측면) 이동**까지 구현하였습니다.
- **실시간 서보모터 각도 제어**  
  - STM32를 이용하여 I2C 통신을 통해  
    보행에 필요한 모든 서보모터의 각도를 **실시간으로 제어**합니다.
- **시뮬레이션 기반 변수 검증**  
  - 파이썬 시뮬레이션을 통해  
    **로봇의 보행 변수(속도, 보폭, 주기 등)**가 실제 움직임에  
    어떤 영향을 주는지 직접 확인하였습니다.
---

## 🖼 시스템 구성도


---

## **소스 코드**

### [소스코드 바로가기]()

## 📽 **시연 영상**

###  [첫 동작](https://drive.google.com/file/d/1HjOwKQp8KJjVj2hTh6_adtx6zlYOwu_e/view?usp=sharing)  
![alt text](gif/First.gif)

###  [전진](https://drive.google.com/file/d/1SwbQgumZ2qXSB3PdyCzM1mTH8wKag_9B/view?usp=sharing)  
![alt text](gif/Straight.gif)

###  [후진](https://drive.google.com/file/d/1MgpPF4JHcQ8GS_9CZfato3qhq4KthBys/view?usp=sharing)
![alt text](gif/Back.gif)

### [좌 이동](https://drive.google.com/file/d/1ZpJAempI7OCzLiTSKFfvauY5EdH2y3WN/view?usp=sharing)
![alt text](gif/Left_Move.gif)

### [우 이동](https://drive.google.com/file/d/1yQKBVOaO4YPHyf00HGkSakdziHmvaDI0/view?usp=sharing)
![alt text](gif/Right_Move.gif)

### [시뮬레이션]()
![alt text]()
