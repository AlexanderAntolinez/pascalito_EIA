# **Pascalito_EIA**

Bienvenido al repositorio de **Pascalito**, un robot móvil desarrollado con ROS 2 que utiliza una Raspberry Pi 5 y hardware adicional para navegación, mapeo y control.

## **Componentes del Robot**
Pascalito está construido con los siguientes componentes de hardware:

- **LiDAR**: RPLIDAR A2M8
- **Computadora principal**: Raspberry Pi 5
- **Controlador**: Yahboom ROS Robot Control Board V1.0
- **Actuadores**: 4 motores DC JGB37-520 con encoders integrados

---

## **Cómo ejecutar Pascalito**

### **1. Mapeo del entorno**
Para iniciar el mapeo desde la estación de control:
```bash
ros2 launch pascalito pascalito_mapeo.py
