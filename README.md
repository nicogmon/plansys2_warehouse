

* **Simulador**
```bash
ros2 launch kobuki simulation_multirobot.launch.py 
```
* **Navegación**
(En namespace se pondrá el namespace de cada robot se debe lazar una navegación por cada robot existente)
```bash
ros2 launch kobuki navigation_multirobot.launch.py namespace:=x_robot
```

* **Navegación Sinteica**
(Utilizar este comando en caso de no querer lanzar la simulación y navegación real. No se deben lanzar los dos anteriores en este caso.)
```bash
ros2 launch plansys2_warehouse navs_launch.py
```
* **plansys2_warehouse Problem**
```bash
ros2 launch plansys2_warehouse warehouse_problem_launch.py 
```

* **Controlador**
```bash
ros2 run plansys2_warehouse_problem warehouse_controller_node
```

Estos comandos deben ejecutarse en terminales independientes. Es recomendable esperar a que finalice la ejecución de cada uno antes del siguiente.

