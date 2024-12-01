# **Pascalito_EIA**

Bienvenido al repositorio de **Pascalito**, un robot móvil desarrollado con ROS 2 Jazzy Jalisco que utiliza una Raspberry Pi 5 y hardware adicional para navegación, mapeo y control.

## **Componentes del Robot**
Pascalito está construido con los siguientes componentes de hardware:

- **LiDAR**: RPLIDAR A1
- **Computadora principal**: Raspberry Pi 5
- **Controlador**: Yahboom ROS Robot Control Board V1.0
- **Actuadores**: 4 motores DC JGB37-520 con encoders integrados

---
## **Instalación de ROS 2 Jazzy Jalisco y Configuración del Entorno**

### **Instalar ROS 2 Jazzy Jalisco**
Sigue los pasos para instalar ROS 2 en Ubuntu 24.04:

### **1.Configure el repositorio de ROS 2:**

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Luego agregue el repositorio a su lista de fuentes:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### **1.1.Instalar herramientas de desarrollo (opcional):**

```bash
sudo apt update && sudo apt install ros-dev-tools
```
### **2.Instalar ROS 2:**
Actualice los cachés de su repositorio apt después de configurar los repositorios.
```bash
sudo apt update
sudo apt upgrade
```
Instalación de escritorio (recomendado): ROS, RViz, demostraciones, tutoriales.
```bash
sudo apt install ros-jazzy-desktop
```
Instalación básica de ROS: bibliotecas de comunicación, paquetes de mensajes, herramientas de línea de comandos. Sin herramientas de interfaz gráfica de usuario.

```bash
sudo apt install ros-jazzy-ros-base
```
### **3.Configura el entorno:**
Agrega ROS 2 al **`bashrc`** para que se cargue al inicio:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---
### **Instalar Dependencias Adicionales para el uso del robot**

Para trabajar con SLAM y Nav2, instala los siguientes paquetes:

```bash
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

---

### **Crear el Espacio de Trabajo**
### **1.Crea un directorio para tu espacio de trabajo:**
```bash
mkdir -p ~/ros2_ws/src
```
### **2.Inicializa el espacio de trabajo:**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### **3.Haz que el espacio de trabajo se cargue automáticamente:**
Agrega esta línea a tu **`~/.bashrc`**:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---
## **Clonar el Paquete del LiDAR**

El paquete utilizado para controlar el LiDAR es **`rplidar_ros`**. Para usarlo en este proyecto, clona el repositorio oficial en tu workspace de ROS 2.

### **Comando para clonar:**
Dirígete a la carpeta **`src`** dentro de tu espacio de trabajo:

```bash
cd ~/ros2_ws/src
```
Usa el comando **`git clone`** seguido de la URL del repositorio:

```bash
git clone https://github.com/Slamtec/rplidar_ros.git
```

Regresa a la raíz del workspace y compílalo con **`colcon`**:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```
Asegúrate de que tu terminal esté utilizando el entorno del espacio de trabajo compilado:

```bash
source install/setup.bash
```
---
## **Clonar el este repositorio para la ejecución de Pascalito**

Los paquetes utilizados para el uso del robot son **`pascalito`** y **`motor_control_pkg`** . Para usarlo en este proyecto, clona el repositorio oficial en tu workspace de ROS 2, pero el segundo paquete debe ser instalado en la RPI 5, este paquete **`yahboomcar_ctrl`** puedes usarlo para el control manual del robot(Revisar las notas).

### **Comando para clonar:**
Dirígete a la carpeta **`src`** dentro de tu espacio de trabajo:

```bash
cd ~/ros2_ws/src
```
Usa el comando **`git clone`** seguido de la URL del repositorio:

```bash
git clone https://github.com/AlexanderAntolinez/pascalito_EIA.git
```

Regresa a la raíz del workspace y compílalo con **`colcon`**:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```
Asegúrate de que tu terminal esté utilizando el entorno del espacio de trabajo compilado:

```bash
source install/setup.bash
```
---
## **Estructura del Proyecto**

Este repositorio contiene:

- **Paquete de simulación (`pascalito_sim`)**: 
  Contiene la simulación del robot usando **SLAM Toolbox** para el mapeo del entorno simulado.

- **Paquete principal (`pascalito`)**: 
  Contiene los nodos y configuraciones principales para el **mapeo** y **navegación**.

- **Paquete de control de motores (`motor_control_pkg`)**: 
  Controla los motores y el LiDAR.

- **Paquete de teleoperación (`yahboomcar_ctrl`)**: 
  Controla el robot por medio del teclado.(ofrecido por los desarroladores de la placa) 

- **Configuraciones de SLAM y navegación**:
  - Archivos de parámetros para **SLAM Toolbox** y **Navigation2**.
  - Archivos de lanzamiento para diferentes funcionalidades del robot.

## **Cómo ejecutar pascalito_sim (NO NECESARIO PARA LA EJECUCIÓN)**
Para iniciar la simulación desde la estación de control:
```bash
ros2 launch pascalito_sim robot_launch.py
```

## **Cómo ejecutar Pascalito desde centro de control**

### **1. Mapeo del entorno**
Para iniciar el mapeo desde la estación de control:
```bash
ros2 launch pascalito pascalito_mapeo.py
```

### **2. Navegación con Nav2**
Para iniciar el sistema de navegación:
```bash
ros2 launch pascalito pascalito_navegante.py
```
### **3. Control para teleoperación (Opcional)**
Para iniciar el sistema de navegación:
```bash
ros2 run yahboomcar_ctrl yahboom_keyboard
```
---

## **Configuración en la Raspberry Pi 5**

### **1. Control de motores y LiDAR**
Para iniciar el mapeo desde la estación de control:
```bash
ros2 launch motor_control_pkg pascalito_bringup.py
```

### **2. Control del LiDAR por separado (Opcional)**
Para iniciar el sistema de navegación:
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```
---

## **Conexión de los motores a la placa Yahboom**

![Pascalito Robot](img/conection.png "Pascalito en acción")

---

## **Notas**

Si vas a utilizar la navegación recuerda que no debes estar ejecutando el **`yahboom_keyboard`**, dado que este manda comandos a /cmd_vel en 0 cuando no estas presionando las teclas





