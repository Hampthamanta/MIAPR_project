# TurtleBot with RRT-Connect path planning algorithm

## 1. Instalacja środowiska z Dockerem

#### 1. Tworzenie środowiska i klonowanie repozytorium

```
mkdir -p ~/turtlebot_ws/src/ && cd ~/turtlebot_ws/src/
```
```
git clone https://github.com/Hampthamanta/MIAPR_project/
```

#### 2. Budowa kontenera

```
cd MIAPR_project && docker build -t nav2_rrt_connect .
```

#### 3. Włączenie kontenera

```
bash start_container.sh 
```
## 2. Konfiguracja i uruchomienie środowiska

#### 1. Podmiana pliku .yaml - wybór plugin plannera

```
cp ~/Shared/src/MIAPR_project/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

#### 2. Budowa środowiska w Dockerze

```
cd ~/Shared/src/MIAPR_project & ./build_env.sh
```

#### 3. Uruchomienie plannera w RVIZ

```
./run_navi.sh
```

## 3. Algorytm RRT-connect w pseudokodzie

![image](https://github.com/user-attachments/assets/2bf54f19-0fb6-48a2-bb3b-e90bb6979d0d)





> [!NOTE]
> Folder `turtlebot_ws` jest udostepniony z konteneru do hosta.

> [!NOTE]
> Dockerfile i skrypty włącząjące kontenery bazują na [Rafał Staszak's repository](https://github.com/RafalStaszak/NIMPRA_Docker/)































------------------------------------------------------------------------
Przygotowanie środowiska:  
* Projekt został wykonany w kontenerze dockera z systemem ubuntu 22 i ROS2 Humlbe  
* Należy zainstalować moduł Nav2 z oficjalnego poradnika:  
  https://docs.nav2.org/getting_started/index.html#running-the-example  

* dodanie do pliku ~/.bashrc:  
export TURTLEBOT3_MODEL=waffle



* Quick FIX zainstalowanie i dodanie do ~/.bashrc (zmiana Fast DDS na Cyclone DDS):  
  apt install ros-humble-rmw-cyclonedds-cpp  
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp    - można dodać do ~/.bashrc lub do pliku przygotowującego środowisko  

  Należy jeszcze upewnić się czy posiadamy odpowiedni typ robota w pliku konfigurtacyjnym:
  sudo gedit /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml
    #robot_model_type: "differential"  
    robot_model_type: "nav2_amcl::DifferentialMotionModel"  


* Sprawdzenie czy działa moduł Nav2:  
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True  

Po uruchomieniu gazebo i rviza, konieczne jest zainicjowanie pozycji początkowej robota, ponieważ Nav2 nie wiem gdie początkowo znajudje się robot.   
Oszacowaną pozycję ustawia się przez kliknięcie w rviz na górnym pasku "2D Pose Estimate" i następnie zaznaczenie na mapie szacowanej pozycji robota wraz z orientacją.  
Wybrana pozycja początkowa nie musi być idealna, ale im dokładniejsza tym szybciej Nav2 poradzi sobie z korekcją właściwej pozycji robota na mapie.  



