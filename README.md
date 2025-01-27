

* **Simulador**
```bash
ros2 launch kobuki simulation.launch.py x:=8.0 y:=-4.0 Y:=-1.57
```
* **Navegación**
```bash
ros2 launch kobuki navigation_sim.launch.py
```
* **Navegación Sinteica**
(Utilizar este comando en caso de no querer lanzar la simulación y navegación real. No se deben lanzar los dos anteriores en este caso.)
```bash
ros2 run plansys2_warehouse_problem nav2_sim_node
```
* **Paquete de plansys2_warehouse**
```bash
ros2 launch plansys2_warehouse_problem warehouse_problem_launch.py 
```

* **Controlador**
```bash
ros2 run plansys2_warehouse_problem warehouse_controller_node
```

Estos comandos deben ejecutarse en terminales independientes. Es recomendable esperar a que finalice la ejecución de cada uno antes del siguiente.

