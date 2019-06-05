# DSBM-PF_CocheRC

## Enunciado del proyecto
Este repositorio contiene el proyecto final de la asignatura Diseño de Sistemas Basados en Microprocesador de la ESI-UCLM. El proyecto consiste en la realización de un programa capaz de controlar un coche mediante microcontroladores y ordenes por Bluetooth a través de una aplicación movil. Además se contará con un sensor de ultrasonidos que nos permitira evitar los posibles obstaculos que se encuentren en la trayectoria del vehículo.

 - Para conectarse al receptor bluetooth depende de la contraseña que tenga, en nuestro caso es 1234.

## Video Demostración
En este video se muestra una demostración del funcionamiento del coche:
https://bit.ly/2WuUtXp

## Versiones
### Versión 0.5
 - Esqueleto del código creado con STM32CubeMX

### Versión 0.8
 - Empezadas funciones de ultrasonidos, motor y bluetooth;
   - Función del motor funciona.
   - Función de ultrasonidos sin probar.
   - Función de bluetooth no recibe.

### Versión 0.9
 - Completadas funciones auxiliares.
   - El cálculo de la distancia con el ultrasonido no funciona bien.
   - La función del bluetooth ya funciona.

### Versión 1.0
 - COCHE EN FUNCIONAMIENTO.
 - La función de ultrasonidos sigue sin calcular bien los valores.

### Versión 1.2
 - Añadido direccionamiento diagonal (Delante-Derecha, Delante-Izquierda, Detras-Derecha, Detras-Izquierda)

### Versión 1.3
 - Añadida variable condicional para el control de obstaculos con ultrasonidos.
   - Los US siguen sin funcionar bien.

### Versión 1.4
 - El ultrasonido ahora calcula bien la distancia.
 - Al detectar un obstaculo se para.

### Versión 1.5
 - Posiciones del motor del ultrasonido ajustadas.

### Versión 1.5.5
 - Añadida primera versión de la documentación.
 - Actualizado el .ioc del proyecto.

### Versión 1.8
 - Reestructuración del código.

### Versión 2.0
 - FUNCIONALIDAD COMPLETA.
   - Ahora el ultrasonido hace bien los cálculos.
 - Añadido esquema de cableado con Fritzing.
  
### Versión 2.1
 - Reestructuración del código.
 - Mejora del tiempo de respuesta.

### Versión 2.2
 - Código comentado.
 - Documentación actualizada.

### Versión 2.5
 - Documentación Finalizada.

### Versión 2.6
 - Documentación Corregida.
