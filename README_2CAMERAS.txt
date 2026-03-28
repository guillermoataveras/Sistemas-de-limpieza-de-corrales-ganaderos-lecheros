# README - `camera_analysis.py`

## Descripción general

El archivo `camera_analysis.py` contiene toda la lógica encargada de:

- abrir y configurar dos cámaras
- capturar frames de ambas cámaras
- analizar los porcentajes de colores por frame
- guardar esos porcentajes en estructuras tipo historial
- dibujar los resultados sobre la imagen
- entregar los porcentajes actuales con un desfase temporal intencional para la segunda cámara

Este archivo **no se encarga de imprimir en consola ni de detectar teclas**.  
Esa responsabilidad queda fuera, normalmente en un archivo como `main.py`.

---

# Objetivo del archivo

La idea de este módulo es separar la lógica de análisis de imagen de la lógica de interacción del programa.

En otras palabras:

- `camera_analysis.py` = procesamiento y datos
- `main.py` = ejecución principal, ventanas, teclado, impresión

Esto hace que el código sea más modular, más fácil de mantener y más fácil de reutilizar en otros proyectos.

---

# Dependencias

Este archivo utiliza las siguientes librerías:

```python
import cv2
import numpy as np
import time
from collections import deque