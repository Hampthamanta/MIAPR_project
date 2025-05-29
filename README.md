# MIAPR_project


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



