# arduino_bridge.py
import time                         # Funciones de tiempo
import threading                    # Para manejar hilos de ejecución en paralelo sin bloquear el programa
import serial                       # Librería para comunicación serial con el Arduino
import serial.tools.list_ports      # Para listar los puertos seriales disponibles en el sistema
from dataclasses import dataclass   # Para definir clases de datos de manera sencilla
from typing import Optional, Callable, Dict, List

# Definición de una clase de datos para representar un mensaje recibido del Arduino
# Con su contenido y el momento en que fue recibido.
@dataclass
class ArduinoMessage:
    raw: str            # Texto completo "crudo" recibido del Arduino
    when: float         # Timestamp (marca de tiempo) de recepción del mensaje
    
    @property
    def label(self) -> str:
        return self.raw.split(":", 1)[0] if ":" in self.raw else self.raw
    
    # Agarra el valor de "raw", busca el primer ":" y devuelve lo que se encuentra antes de ese ":" como etiqueta (label). 
    # Si no hay ":", devuelve el mensaje completo como etiqueta. 
    
    # Por ejemplo, si el mensaje "raw" es "TEMP: 25.3", el label sería "TEMP". 
    # Si el mensaje es simplemente "HELLO", el label sería "HELLO".
    
    @property
    def content(self) -> str: 
        return self.raw.split(":", 1)[1] if ":" in self.raw else ""
    
    # Extrae todo lo que se encuentra después del primer ":" en el mensaje "raw".
    # Por ejemplo, si el mensaje "raw" es "TEMP: 25.3", el content sería " 25.3".
    # Si el mensaje es simplemente "HELLO", el content sería una cadena vacía ""
    
    def as_floats(self) -> List[float]:
        try: 
            # 1. Toma el contenido que se extrajo de "raw", por ejemplo: : "12.5, 0.0, -1.2"
            # 2. Reemplaza las comas por espacios para unificar el separador, quedando: "12.5 0.0 -1.2"
            # 3. Divide la cadena resultante por espacios, obteniendo una lista de partes: ["12.5", "0.0", "-1.2"]
            # 4. Convierte cada parte de la lista en un número flotante, resultando en: [12.5, 0.0, -1.2]
            
            return [float(p) for p in self.content.replace(",", " ").split()]
        
        except ValueError: 
            # En caso de que alguna parte no pueda convertirse a flotante (por ejemplo, si el contenido no es numérico)
            # Se captura la excepción ValueError y se devuelve una lista vacía.
            return []

# Definición de la clase principal que maneja la conexión con el Arduino, la lectura de mensajes y el envío de comandos.
class ArduinoLink:
    def __init__(self, port=None, baud=115200):
        # Puerto serial al que esta conectado el Arduino y el Baudrate
        self.port = port            
        self.baud = baud       
             
        self.ser = None             # Objeto de la conexión serial, inicialmente None porque aún no se ha establecido la conexión        
        self.connected = False      # Flag que indica el estado de la conexión, inicialmente False porque no se ha conectado aún
        self._handlers = {}         # Diccionario para almacenar los handlers de mensajes, donde la clave es la etiqueta (label)
                                    # y el valor es una lista de funciones que manejan esa etiqueta

    # Función para registrar un handler de mensajes para una etiqueta específica.
    def on_message(self, label: str):
        # Decorador que se utiliza para asociar una función a una etiqueta de mensaje específica.
        def decorator(func):
            # Si la etiqueta no existe en el diccionario de handlers, se crea una nueva entrada con una lista vacía.
            if label not in self._handlers: self._handlers[label] = []
            
            # Añade la función proporcionada a la lista de handlers para esa etiqueta.
            self._handlers[label].append(func)
            
            # Devuelve la función original sin modificarla.
            return func
        
        # Devuelve el decorador.
        return decorator
    
    
    # Función para iniciar el proceso de conexión y lectura de mensajes del Arduino.
    def start(self):
        # Inicia un hilo separado para manejar la conexión y lectura de mensajes del Arduino sin bloquear el programa principal.
        threading.Thread(target=self._manager_loop, daemon=True).start()

    # Función que posee un bucle infinito para gestionar la conexión con el Arduino. 
    # Si no está conectado, intenta establecer la conexión.
    def _manager_loop(self):
        while True:
            if not self.connected:
                self._attempt_connect()
            time.sleep(1.0)
            
    # Funcion que intenta establecer la conexión con el Arduino.
    def _attempt_connect(self):
        target = self.port or self._find_port() # Busca el puerto especificado o intenta encontrarlo automáticamente
        if not target: return                   # Si no se encuentra ningún puerto, sale de la función y espera para intentar de nuevo
        
        try:
            self.ser = serial.Serial(target, self.baud, timeout=0.1)        # Abre el puerto a un baudrate especifico con un timeout de lectura
            self.ser.dtr = True                                             # Activa la línea DTR para resetear el Arduino, lo que es común para iniciar la comunicación
            time.sleep(2.0)                                                 # Espera al booteo
            self.connected = True                                           # Marca la conexión como establecida
            threading.Thread(target=self._reader_loop, daemon=True).start() # Empieza el hilo para leer el puerto
        except: self.connected = False                                      # Ante cualquier error marca desconectado

    # Funcion que corre en un hilo separado para leer continuamente los datos del puerto serial.
    def _reader_loop(self):
        buf = b""
        while self.connected:   # Mientras se este conectado
            try:
                chunk = self.ser.read(256)  # Lee hasta 256 bytes del puerto serial. 
                if not chunk: continue      # Si no se recibe ningun dato, continua el bucle
                buf += chunk                # Acumula bytes en el buffer
                
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode(errors="replace").strip()
                    if text: self._dispatch(ArduinoMessage(text, time.time()))
            except: self.connected = False

    def _dispatch(self, msg):
        for handler in self._handlers.get(msg.label, []): handler(msg)l
        for handler in self._handlers.get("*", []): handler(msg)

    def send(self, cmd: str):
        if self.connected:
            try: self.ser.write(f"{cmd}\n".encode())
            except: self.connected = False

    def _find_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if any(k in f"{p.description} {p.manufacturer}".lower() for k in ["arduino", "ch340", "usb"]):
                return p.device
        return None