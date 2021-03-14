# EPD 1-3 Robótica

## Realizado por:

* Pedro Martín Sánchez
* Juan Carlos Ruiloba Calderón

### 1. Introducción

Nuestro grupo ha optado por resolver estas epds, usando un algorimo de bug, en el que siempre intentará esquivar un obstáculo bordeándolo hasta intentar llegar al objetivo. Así mismo este robot sigue un camino descrito en el archivo *config/path.yml*. Ademas estará pendiente de los subcriptores que le hemos añadido, para los que el robot atenderá al instante y tras completarlos continuará con el camino previsto anteriormente. 

* Objetivos
  * Crear un programa que de manera autónoma el robot (Turtlebot) alcance un destino siguiendo un camino evitando obstáculos.
  * Usar los sensores láser de distancia integrados en el robot para detectar obstáculos y comandar al robot hasta el destino siguiendo un camino dado.
  * Uso de los subcriptores para recibir al instante unos puntos y/o caminos que deberá alcanzar.

### 2. Instrucciones de Ejecución

Nuestro programa por defecto carga dos puntos que están definidos en el fichero *config/path.yml*. Para ello hay que ejecutar el comando ```roslaunch epd3 epd3.launch```.<br>Para utilizar el **PoseStamped subscriber**, hay indicar un punto nuevo a través de la herramienta **Rviz** con la herramienta **2D Nav Goal**. El robot irá directamente a dicho punto, aunque esté moviéndose hacia otro punto. Después de haber llegado a dicho punto, seguirá su camino anterior.<br>Para utilizar el **Path subscriber** hay que ejecutar el comando ```rosrun epd3 followingPath.py``` en una consola aparte. Para introducir el camino, hay que introducirlo por consola siguiendo esta sintaxis: "x1,y1;x2,y2".

### 3. Dificultades:

+ EPD 1
  <br>Al estar la epd desactualizada, tuvimos que buscar en internet cómo instalar la versión de ROS melodic.
  <br>Y al intentar esquivar los obstáculos, habiamos planteado que al detectarlos fuera el robot marcha atrás y a la vez girase, pero no terminamos de conseguir que los esquivase como queríamos, así que al final optamos por limitar el campo de visión del robot, consiguiendo que funcionara como queríamos esquivando los obstáculos hacia el mismo sentido (algoritmo de bug).
+ EPD 2
  <br>No tuvimos ningún problema al realizar esta epd.
+ EPD 3
  <br>La única dificultad reseñable fue conseguir que funcionase el suscriptor del camino. Hasta que no aclaró el profesor cómo podíamos hacerlo no supimos el modo de resolverlo.
