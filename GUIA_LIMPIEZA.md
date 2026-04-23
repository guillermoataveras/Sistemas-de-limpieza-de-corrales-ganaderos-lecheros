# Guía de operación — Robot limpiador de corrales lecheros

Flujo paso a paso desde el encendido hasta el cierre de sesión para ejecutar una limpieza exitosa.

## 1. Preparación física

1. Posicionar el robot en el centro del corral, preferiblemente mirando hacia el norte del mapa (eje +Y).
2. Verificar que el LiDAR gire libremente y que ambas cámaras (frontal e índice 2 trasera) tengan visión despejada.
3. Conectar:
   - Arduino Mega en `COM7`
   - LiDAR LD20 en `COM8`
   - Cámara frontal (índice 1)
   - Cámara trasera (índice 2)
4. Tanque de agua lleno y compresor operativo.

## 2. Arranque del software

5. Ejecutar en terminal:
   ```bash
   python LiDar3_Control_Program.py
   ```
6. Si aparece el modal de sesión previa:
   - `S` para restaurar la sesión guardada
   - `N` para iniciar una sesión nueva
7. Esperar en consola el mensaje:
   ```
   [REF SQUARE] Centro fijado en (X, Y) mm — lado 2.2m
   ```

## 3. Calibración inicial (modo MANUAL)

8. Verificar visualmente que el cuadrado cian de 2.2×2.2 m aparece alrededor del robot en el mapa.
9. Presionar **F6** — fuerza un scan match inmediato. En consola debe aparecer:
   ```
   [MATCH F6] dx=... dy=... dyaw=... pairs=... q=XX% walls=4/4
   ```
   La calidad (`q`) debe ser superior a 60% y preferiblemente las 4 paredes detectadas.
10. Si la nube LiDAR no se alinea con el cuadrado después de F6, detener el programa, editar la constante `STARTUP_YAW_OFFSET_DEG` en el código, y reiniciar.
11. (Opcional) Presionar **Z** para recalibrar el yaw a 0° desde la orientación física actual.

## 4. Escaneo de paredes del corral

12. Presionar **F1** — el robot queda quieto acumulando hits del LiDAR durante 5–10 segundos.
13. Esperar mensaje:
    ```
    [SCAN] Completado — N celdas de pared
    ```
14. Verificar que las paredes detectadas (líneas grises en el mapa) coinciden visualmente con el corral real.

## 5. Configurar la misión

15. Presionar **Tab** — cambiar a modo RUTA (pantalla azul).
16. Presionar **R** — activar sistema de limpieza (bomba y cepillos se activarán automáticamente durante la ruta).
17. Opcional: presionar **O** — activar compresor para alta presión constante.
18. Clic en botón **🏠 GUARDAR HOME AQUÍ** del panel — guarda la posición actual para retorno al terminar.
19. Presionar **F2** hasta ver el patrón deseado en el panel. Recomendado: **SNAKE-90** (giros de 90° en lugar de 180°).
20. (Opcional) Zonas prohibidas: presionar **N** y arrastrar el mouse sobre el mapa para dibujar rectángulos no-go.
21. Presionar **F3** — generar o recalcular la ruta. Los waypoints aparecen en el mapa.

## 6. Durante la limpieza

22. El robot arranca automáticamente siguiendo la FSM:
    ```
    ROTATE → ROTATE_SETTLE → ROTATE → ... → ADVANCE → WP_REACHED → (siguiente WP)
    ```
23. Monitorear pills del toolbar:
    - `⌖ match XX%` debe mantenerse sobre 30% la mayor parte del tiempo
    - Si aparece `⚠ POSE INCIERTA` en rojo de forma persistente, detener y revisar el LiDAR
24. Ante un obstáculo, el robot entra en estado `BLOCKED_ROTATE` (naranja) y rota 90° hasta encontrar camino despejado.
25. Controles durante operación:
    - **X** — stop total inmediato
    - **Tab** — volver a modo manual
    - **V** — ciclar potencia de limpieza
    - **1 / 2** — ajustar PWM de avance ±
    - **3 / 4** — ajustar PWM de giro ±
    - **[ / ]** — ajustar umbral de cobertura ± 5%

## 7. Finalización

26. Al alcanzar la cobertura objetivo, el robot entra automáticamente en `RETURN_HOME` (si hay home guardado) y regresa al punto inicial usando A* + ROTATE/ADVANCE (nunca retrocede).
27. Al llegar al home, el robot pasa a modo MANUAL y se detiene.
28. Presionar **M** — exportar mapa PNG con la cobertura final.
29. Presionar **L** — exportar log CSV de la sesión.
30. Cerrar con la **X** de la ventana. El estado se guarda automáticamente.

## Atajos de emergencia

| Tecla | Acción |
|---|---|
| **X** | Stop total inmediato |
| **Shift + E** | Toggle relé de emergencia (corta compresor) |
| **Backspace** | Limpiar todos los waypoints |
| **P** | Reset completo del runtime |

## Referencia rápida de teclas

| Categoría | Tecla | Acción |
|---|---|---|
| Modo | **Tab** | Alternar manual / ruta |
| Movimiento | **W / S / A / D** | Avanzar / retroceder / girar izq / der |
| Escaneo | **F1** | Escanear paredes |
| Patrón | **F2** | Ciclar patrón de limpieza |
| Ruta | **F3** | Replan |
| Vista | **F4** | Toggle cuadrado de referencia |
| Vista | **F5** | Reset rotación visual del mapa |
| Pose | **F6** | Scan match inmediato (XY + yaw) |
| Calibración | **Z** | Recalibrar yaw actual como 0° |
| Limpieza | **R** | Toggle limpieza automática |
| Limpieza | **O** | Toggle compresor |
| Limpieza | **V** | Cambiar potencia |
| Limpieza | **B** | Override manual bomba |
| Limpieza | **C** | Override manual cepillos |
| No-go | **N** | Modo dibujo de zona prohibida |
| No-go | **Delete** | Borrar última zona no-go |
| Export | **M** | Exportar mapa PNG |
| Export | **L** | Exportar log CSV |
| Panel | **7 / 8 / 9 / 0** | Expandir / colapsar secciones |
| Home | **Clic en botón** | Guardar pose como home |
| Reset | **Backspace** | Borrar waypoints |
| Reset | **P** | Reset runtime completo |

## Notas técnicas

- **Frecuencia de match**: cada tick (30 Hz) contra las 4 paredes del cuadrado de referencia, pausado sólo durante giros fuertes (diferencial de PWM ≥ 60).
- **Estrategia de giros**: bursts de 300 ms seguidos de pausas de 300 ms para que el match converja; tolerancia de alineación 3°.
- **Escape de obstáculos**: rotaciones de 90° hasta encontrar camino despejado (máximo 4 intentos = giro completo). Nunca retrocede.
- **Detección de suciedad**: píxeles blancos (alto V, baja S en HSV) se interpretan como SUCIO; píxeles oscuros o con color como LIMPIO.
